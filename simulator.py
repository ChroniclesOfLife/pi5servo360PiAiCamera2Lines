import sys
import math
import pygame
from robot_logic import RobotLogic

# Setup Pygame colors and dimensions
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
DARK_RED = (200, 0, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
YELLOW = (0, 255, 255)
GUI_BG = (50, 50, 50)
WIDTH, HEIGHT = 1000, 600 # 800 for map, 200 for GUI
MAP_WIDTH = 800

class SimulatedCar:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle = 0  # 0 degrees is facing East (right)
        self.width = 40
        self.height = 20
        self.base_speed = 3.0
        
        # Virtual sensors offsets: (dx, dy) relative to car center when facing 0 deg
        self.sensor_dist = 25 # Distance in front of car center
        self.sensor_spread = 20 # Distance from center line
        
    def move(self, pid_output):
        """ Update position and angle based on base speed and PID output (steering) """
        # Limit max steering to simulate physical capabilities (like hardware BASE_SPEED)
        turn_adjust = max(-3.0, min(3.0, pid_output))
        self.angle -= turn_adjust
        
        # Calculate new X and Y based on angle
        rad = math.radians(self.angle)
        self.x += self.base_speed * math.cos(rad)
        self.y -= self.base_speed * math.sin(rad) # Y is inverted in Pygame
        
    def get_camera_error(self, track_surface, get_pixel_color):
        """ Simulates a camera reading a row of pixels and finding the line center """
        scan_width = 160  # total pixels to scan
        rad = math.radians(self.angle)
        
        # Center of the "camera" line
        cx = self.x + self.sensor_dist * math.cos(rad)
        cy = self.y - self.sensor_dist * math.sin(rad)
        
        # Direction perpendicular to car (Right is positive i)
        perp_rad = rad - math.pi/2
        dx = math.cos(perp_rad)
        dy = -math.sin(perp_rad)
        
        black_pixels = []
        scan_points = []
        center_pt = None
        detection_region = []  # Region where detection was found
        
        for i in range(int(-scan_width/2), int(scan_width/2)):
            px = int(cx + i * dx)
            py = int(cy + i * dy)
            scan_points.append((px, py))
            
            color = get_pixel_color(track_surface, px, py)
            if color[0] < 100:  # Dark pixel
                black_pixels.append(i)
                detection_region.append((px, py))
                
        error = None
        if len(black_pixels) > 0:
            error = sum(black_pixels) / len(black_pixels)
            # Find actual center point for drawing
            center_pt = (int(cx + error * dx), int(cy + error * dy))
            
        return error, scan_points, center_pt, detection_region

def draw_track(surface):
    surface.fill(WHITE)
    # Draw two nested rectangles (a simple track track)
    pygame.draw.rect(surface, BLACK, (100, 100, 600, 400), width=30)
    
def get_pixel_color(surface, x, y):
    """ Safe pixel retrieval with bounds checking """
    if 0 <= x < MAP_WIDTH and 0 <= y < HEIGHT:
        return surface.get_at((x, y))
    return WHITE # Out of bounds is "white"

def main(slow_motion_factor=1, debug_verbose=True):
    """
    Launch the digital twin simulator.
    
    Args:
        slow_motion_factor (int): Divide frame rate by this to slow down (2 = half speed)
        debug_verbose (bool): Print extra debug info to console
    """
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(f"Robot Digital Twin Simulator [Slow Motion x{slow_motion_factor}]")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 18)
    small_font = pygame.font.SysFont("Arial", 14)
    
    # Track surface (drawn once, used for sensor collision)
    track_surface = pygame.Surface((MAP_WIDTH, HEIGHT))
    draw_track(track_surface)
    
    # Initialize Core Logic and Car
    logic = RobotLogic(kp=0.5, kd=0.1) # Tuned slightly for sim
    car = SimulatedCar(x=150, y=115) # Start on the top track edge, facing right
    car.angle = 0 # Facing East

    frame_count = 0
    last_ui_update = 0
    ui_text = []
    
    # Target frame rate with slow motion
    target_fps = 60 // slow_motion_factor
    
    print(f"[Simulator] Starting with {slow_motion_factor}x slow motion (target {target_fps} FPS)")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_p:
                    # Pause on 'P'
                    print(f"[PAUSED at Frame {frame_count}] Press P to resume or ESC to quit")
                    paused = True
                    while paused:
                        for pause_event in pygame.event.get():
                            if pause_event.type == pygame.QUIT:
                                running = False
                                paused = False
                            if pause_event.type == pygame.KEYDOWN:
                                if pause_event.key == pygame.K_p:
                                    paused = False
                                    print(f"[RESUMED]")
                                if pause_event.key == pygame.K_ESCAPE:
                                    running = False
                                    paused = False
                
        # --- SENSOR LOGIC (VIRTUAL CAMERA) ---
        # Updated signature: get_camera_error now returns 4 values
        error, scan_points, center_pt, detection_region = car.get_camera_error(track_surface, get_pixel_color)
        
        # --- CORE BACKEND LOGIC ---
        pid_output, reasoning, p_term, d_term = logic.calculate_pid(error)
        state = logic.update_state(horizontal_line_detected=False, trigger_zone_reached=False) # Simplified for basic following
        logic.log_data(frame_count, error, p_term, d_term, pid_output, reasoning)
        
        # --- MOVE CAR ---
        car.move(pid_output)
        
        # --- RENDER MAP ---
        screen.blit(track_surface, (0,0))
        
        # ========== DETECTION VISUALIZATION (RED AREAS) ==========
        # Draw detection region in RED with transparency
        if len(detection_region) > 2:
            pygame.draw.circle(screen, RED, center_pt if center_pt else (0,0), 25)
            # Draw filled detection area
            pygame.draw.lines(screen, DARK_RED, detection_region, 4)
        
        # Draw Car (Box) - more prominent
        car_rect = pygame.Surface((car.width, car.height), pygame.SRCALPHA)
        car_rect.fill(RED)
        rotated_car = pygame.transform.rotate(car_rect, car.angle)
        rect = rotated_car.get_rect(center=(car.x, car.y))
        screen.blit(rotated_car, rect.topleft)
        
        # Draw car direction indicator (arrow)
        rad = math.radians(car.angle)
        arrow_len = 20
        arrow_x = car.x + arrow_len * math.cos(rad)
        arrow_y = car.y - arrow_len * math.sin(rad)
        pygame.draw.line(screen, YELLOW, (car.x, car.y), (arrow_x, arrow_y), 2)
        
        # Draw Virtual Camera Scanline
        if len(scan_points) > 1:
            pygame.draw.lines(screen, CYAN, False, scan_points, 2) # Cyan line for scan
        if center_pt:
            pygame.draw.circle(screen, GREEN, center_pt, 8) # Larger green dot at detected center
            pygame.draw.circle(screen, WHITE, center_pt, 8, 1)  # White outline
            
        # Draw center of camera line (where error = 0)
        cx = car.x + car.sensor_dist * math.cos(math.radians(car.angle))
        cy = car.y - car.sensor_dist * math.sin(math.radians(car.angle))
        pygame.draw.circle(screen, YELLOW, (int(cx), int(cy)), 5)
        
        # Draw sensor field annotations
        font_tiny = pygame.font.SysFont("Arial", 10)
        sensor_label = font_tiny.render("CAMERA", True, WHITE)
        screen.blit(sensor_label, (int(cx)-15, int(cy)-15))
        
        # --- RENDER GUI DASHBOARD ---
        pygame.draw.rect(screen, GUI_BG, (MAP_WIDTH, 0, WIDTH-MAP_WIDTH, HEIGHT))
        
        # Only update GUI text values 10 times a sec for readability
        current_time = pygame.time.get_ticks()
        if current_time - last_ui_update > 100:
            last_ui_update = current_time
            
            # Break reasoning string into multiple lines for display
            reasoning_lines = reasoning.split(" | ") if reasoning else ["No Data"]
            
            ui_text = [
                f"=== SIMULATOR DEBUG ===",
                f"Frame: {frame_count}",
                f"FPS: {target_fps} (1x={int(60/slow_motion_factor)})",
                f"State: {state}",
                f"Detection: {'✓ YES' if center_pt else '✗ LOST'}",
                f"Car Angle: {car.angle:.1f}°",
                f"Car Pos: ({car.x:.0f}, {car.y:.0f})",
                "",
                f"=== PID ALGORITHM ===",
                f"Error: {error:.2f}px" if error is not None else "Error: LOST",
                f"P_term (Kp={logic.kp}): {p_term:.3f}",
                f"D_term (Kd={logic.kd}): {d_term:.3f}",
                f"Total PID: {pid_output:.3f}",
                f"Turn Adjust: {max(-3.0, min(3.0, pid_output)):.3f}",
                "",
                f"=== REASONING ===",
            ]
            ui_text.extend(reasoning_lines)
            
        for i, text in enumerate(ui_text):
            # Highlight important decisions
            color = WHITE
            if "LOST" in text:
                color = RED
            elif "✓ YES" in text:
                color = GREEN
            elif text.startswith("="):
                color = CYAN
            elif "DECISION" in text or "->" in text:
                color = YELLOW
                
            text_surface = font.render(text, True, color) if not text.startswith("===") else font.render(text, True, color)
            screen.blit(text_surface, (MAP_WIDTH + 10, 20 + i * 20))
            
        # Draw legend at bottom
        legend_y = HEIGHT - 80
        pygame.draw.rect(screen, (30, 30, 30), (MAP_WIDTH, legend_y, WIDTH-MAP_WIDTH, 80))
        
        legend_items = [
            ("Cyan Line", CYAN),
            ("Green ●", GREEN),
            ("Red Area", RED),
            ("Yellow Arrow", YELLOW),
        ]
        
        for idx, (label, color) in enumerate(legend_items):
            x = MAP_WIDTH + 10
            y = legend_y + 10 + (idx % 2) * 25
            if idx >= 2:
                x += 150
            pygame.draw.circle(screen, color, (x-5, y), 3)
            text_surface = small_font.render(label, True, WHITE)
            screen.blit(text_surface, (x + 5, y - 8))
            
        pygame.display.flip()
        clock.tick(target_fps)  # Respect slow motion factor
        frame_count += 1

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    # Run with 2x slow motion for better visualization and debugging
    # Change slow_motion_factor to 1 for real-time (60 FPS)
    main(slow_motion_factor=2, debug_verbose=True)
