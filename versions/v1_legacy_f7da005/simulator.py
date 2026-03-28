import sys
import math
import pygame
from robot_logic import RobotLogic

# Setup Pygame colors and dimensions
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
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
        
        for i in range(int(-scan_width/2), int(scan_width/2)):
            px = int(cx + i * dx)
            py = int(cy + i * dy)
            scan_points.append((px, py))
            
            color = get_pixel_color(track_surface, px, py)
            if color[0] < 100:  # Dark pixel
                black_pixels.append(i)
                
        error = None
        if len(black_pixels) > 0:
            error = sum(black_pixels) / len(black_pixels)
            # Find actual center point for drawing
            center_pt = (int(cx + error * dx), int(cy + error * dy))
            
        return error, scan_points, center_pt

def draw_track(surface):
    surface.fill(WHITE)
    # Draw two nested rectangles (a simple track track)
    pygame.draw.rect(surface, BLACK, (100, 100, 600, 400), width=30)
    
def get_pixel_color(surface, x, y):
    """ Safe pixel retrieval with bounds checking """
    if 0 <= x < MAP_WIDTH and 0 <= y < HEIGHT:
        return surface.get_at((x, y))
    return WHITE # Out of bounds is "white"

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Robot Digital Twin Simulator")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 18)
    
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

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
        # --- SENSOR LOGIC (VIRTUAL CAMERA) ---
        error, scan_points, center_pt = car.get_camera_error(track_surface, get_pixel_color)
        
        # --- CORE BACKEND LOGIC ---
        pid_output, reasoning, p_term, d_term = logic.calculate_pid(error)
        state = logic.update_state(horizontal_line_detected=False, trigger_zone_reached=False) # Simplified for basic following
        logic.log_data(frame_count, error, p_term, d_term, pid_output, reasoning)
        
        # --- MOVE CAR ---
        car.move(pid_output)
        
        # --- RENDER MAP ---
        screen.blit(track_surface, (0,0))
        
        # Draw Car (Box)
        car_rect = pygame.Surface((car.width, car.height), pygame.SRCALPHA)
        car_rect.fill(RED)
        rotated_car = pygame.transform.rotate(car_rect, car.angle)
        rect = rotated_car.get_rect(center=(car.x, car.y))
        screen.blit(rotated_car, rect.topleft)
        
        # Draw Virtual Camera Scanline
        if len(scan_points) > 1:
            pygame.draw.lines(screen, (200, 200, 0), False, scan_points, 2) # Yellow line
        if center_pt:
            pygame.draw.circle(screen, GREEN, center_pt, 6) # Green dot at detected center
            
        # Draw center of camera line (where error = 0)
        cx = car.x + car.sensor_dist * math.cos(math.radians(car.angle))
        cy = car.y - car.sensor_dist * math.sin(math.radians(car.angle))
        pygame.draw.circle(screen, RED, (int(cx), int(cy)), 3)
        
        # --- RENDER GUI DASHBOARD ---
        pygame.draw.rect(screen, GUI_BG, (MAP_WIDTH, 0, WIDTH-MAP_WIDTH, HEIGHT))
        
        # Only update GUI text values 10 times a sec for readability
        current_time = pygame.time.get_ticks()
        if current_time - last_ui_update > 100:
            last_ui_update = current_time
            
            # Break reasoning string into multiple lines for display
            reasoning_lines = reasoning.split(" | ") if reasoning else ["No Data"]
            
            ui_text = [
                f"--- Dashboard ---",
                f"Frame: {frame_count}",
                f"State: {state}",
                f"Line Detect: {'Yes' if center_pt else 'Lost'}",
                f"Angle: {car.angle:.1f}",
                "",
                f"--- PID Math ---",
                f"Error Input: {error:.1f}" if error is not None else "Error Input: Lost",
                f"P Term: {p_term:.2f}",
                f"D Term: {d_term:.2f}",
                f"Total PID: {pid_output:.2f}",
                "",
                f"--- Reasoning ---"
            ]
            ui_text.extend(reasoning_lines)
            
        for i, text in enumerate(ui_text):
            # Highlight important decisions
            color = WHITE
            if text.startswith("-> QUYẾT ĐỊNH: Rẽ"):
                 color = (255, 255, 0) # Yellow
            text_surface = font.render(text, True, color)
            screen.blit(text_surface, (MAP_WIDTH + 10, 20 + i * 25))
            
        pygame.display.flip()
        clock.tick(60) # 60 FPS
        frame_count += 1

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
