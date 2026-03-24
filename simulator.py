import sys
import os
import math
import random
import pygame

# Keep local imports working when launched via external wrappers.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from robot_logic import RobotLogic
from robot_brain import RobotBrain, SensorObservation, Pose, Heading, BrainState, MapSpec, ActionType

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

# Integration toggles for safe A/B testing.
USE_NEW_BRAIN = True
WAIT_FOR_4_POINTS = True
DEFAULT_HEADING = Heading.E

# Window-pane geometry on the map panel (left 800 px).
# Robot drives on the centerline between OUTER and INNER borders.
OUTER_LEFT = 70
OUTER_TOP = 70
OUTER_WIDTH = 660
OUTER_HEIGHT = 460

LANE_WIDTH_PX = 120
INNER_LEFT = OUTER_LEFT + LANE_WIDTH_PX
INNER_TOP = OUTER_TOP + LANE_WIDTH_PX
INNER_WIDTH = OUTER_WIDTH - 2 * LANE_WIDTH_PX
INNER_HEIGHT = OUTER_HEIGHT - 2 * LANE_WIDTH_PX
BORDER_WIDTH = 3
LANE_WIDTH = 20
TRACK_DARK_THRESHOLD = 100
CLICK_SNAP_RADIUS = 16
EDGE_MARGIN_PX = 10
CORRIDOR_CLAMP_PX = LANE_WIDTH_PX * 0.34
TURN_CORRIDOR_CLAMP_PX = LANE_WIDTH_PX * 0.24

MAP_CENTER_X = MAP_WIDTH / 2.0
MAP_CENTER_Y = HEIGHT / 2.0

# Centerline rectangle where the vehicle actually drives.
ROAD_LEFT = (OUTER_LEFT + INNER_LEFT) / 2.0
ROAD_TOP = (OUTER_TOP + INNER_TOP) / 2.0
ROAD_WIDTH = (OUTER_WIDTH + INNER_WIDTH) / 2.0
ROAD_HEIGHT = (OUTER_HEIGHT + INNER_HEIGHT) / 2.0


def screen_to_brain(pt):
    """Convert screen coordinates to RobotBrain map coordinates (origin at center, +Y upward)."""
    x, y = pt
    return (x - MAP_CENTER_X, MAP_CENTER_Y - y)


def brain_to_screen(pt):
    """Convert RobotBrain map coordinates to screen coordinates."""
    x, y = pt
    return (x + MAP_CENTER_X, MAP_CENTER_Y - y)


def project_brain_to_canonical_path(pt, half_w, half_h):
    """Project a brain-space point onto the nearest canonical segment and return (point, distance_px)."""
    x, y = pt
    segments = [
        ((-half_w, half_h), (half_w, half_h)),
        ((-half_w, 0.0), (half_w, 0.0)),
        ((-half_w, -half_h), (half_w, -half_h)),
        ((-half_w, -half_h), (-half_w, half_h)),
        ((0.0, -half_h), (0.0, half_h)),
        ((half_w, -half_h), (half_w, half_h)),
    ]

    best_pt = None
    best_d2 = float("inf")

    for (x1, y1), (x2, y2) in segments:
        vx = x2 - x1
        vy = y2 - y1
        seg_len2 = vx * vx + vy * vy
        if seg_len2 <= 1e-12:
            qx, qy = x1, y1
        else:
            t = ((x - x1) * vx + (y - y1) * vy) / seg_len2
            t = max(0.0, min(1.0, t))
            qx = x1 + t * vx
            qy = y1 + t * vy

        d2 = (x - qx) ** 2 + (y - qy) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_pt = (float(qx), float(qy))

    return best_pt, math.sqrt(best_d2)


def snap_brain_to_canonical_path(pt, half_w, half_h, tol=LANE_WIDTH):
    """Project a brain-space point onto nearest canonical path line used by RobotBrain graph."""
    snapped, distance = project_brain_to_canonical_path(pt, half_w, half_h)
    if snapped is None or distance > tol:
        return None
    return snapped


def screen_to_canonical_brain(pt, map_spec):
    """Convert screen point to brain coordinates and snap to canonical path lines."""
    raw = screen_to_brain(pt)
    snapped = snap_brain_to_canonical_path(
        raw,
        map_spec.half_width,
        map_spec.half_height,
        tol=LANE_WIDTH,
    )
    return snapped


def heading_to_angle(heading):
    return {
        Heading.E: 0.0,
        Heading.N: 90.0,
        Heading.W: 180.0,
        Heading.S: 270.0,
    }[heading]


def random_spawn_on_track(track_surface):
    """Sample a random drivable (dark) pixel for initial robot drop."""
    if USE_NEW_BRAIN:
        hw = ROAD_WIDTH / 2.0
        hh = ROAD_HEIGHT / 2.0
        segs = [
            # horizontal segments
            (MAP_CENTER_X - hw, MAP_CENTER_X + hw, MAP_CENTER_Y - hh),
            (MAP_CENTER_X - hw, MAP_CENTER_X + hw, MAP_CENTER_Y),
            (MAP_CENTER_X - hw, MAP_CENTER_X + hw, MAP_CENTER_Y + hh),
            # vertical segments
            (MAP_CENTER_X - hw, MAP_CENTER_Y - hh, MAP_CENTER_Y + hh),
            (MAP_CENTER_X, MAP_CENTER_Y - hh, MAP_CENTER_Y + hh),
            (MAP_CENTER_X + hw, MAP_CENTER_Y - hh, MAP_CENTER_Y + hh),
        ]
        import random
        idx = random.randint(0, 5)
        seg = segs[idx]
        if idx < 3: # horizontal
            x = random.uniform(seg[0], seg[1])
            y = seg[2]
        else: # vertical
            x = seg[0]
            y = random.uniform(seg[1], seg[2])
        return x, y

    for _ in range(5000):
        x = random.randint(0, MAP_WIDTH - 1)
        y = random.randint(0, HEIGHT - 1)
        color = get_pixel_color(track_surface, x, y)
        if color[0] < TRACK_DARK_THRESHOLD:
            return float(x), float(y)
    # Safe fallback on map center line.
    return float(MAP_CENTER_X), float(MAP_CENTER_Y)


def find_nearest_track_point(track_surface, x, y, radius=CLICK_SNAP_RADIUS):
    """
    Finds the closest canonical track point to the user's click.
    Since visual track is just hollow lanes, we search mathematically using map_spec if USE_NEW_BRAIN.
    """
    if USE_NEW_BRAIN:
        hw = ROAD_WIDTH / 2.0
        hh = ROAD_HEIGHT / 2.0
        candidates = []
        for y_line in (MAP_CENTER_Y, MAP_CENTER_Y - hh, MAP_CENTER_Y + hh):
            px = max(MAP_CENTER_X - hw, min(MAP_CENTER_X + hw, x))
            candidates.append((px, y_line))
        for x_line in (MAP_CENTER_X, MAP_CENTER_X - hw, MAP_CENTER_X + hw):
            py = max(MAP_CENTER_Y - hh, min(MAP_CENTER_Y + hh, y))
            candidates.append((x_line, py))
            
        best = min(candidates, key=lambda p: (p[0] - x)**2 + (p[1] - y)**2)
        if (best[0] - x)**2 + (best[1] - y)**2 <= radius**2:
            return best
        return None

    # Fallback for old logic
    cx = int(round(x))
    cy = int(round(y))
    best = None
    best_d2 = None

    x0 = max(0, cx - radius)
    x1 = min(MAP_WIDTH - 1, cx + radius)
    y0 = max(0, cy - radius)
    y1 = min(HEIGHT - 1, cy + radius)

    for py in range(y0, y1 + 1):
        for px in range(x0, x1 + 1):
            d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy)
            if d2 > radius * radius:
                continue

            color = get_pixel_color(track_surface, px, py)
            if color[0] < TRACK_DARK_THRESHOLD:
                if best is None or d2 < best_d2:
                    best = (float(px), float(py))
                    best_d2 = d2

    return best

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
        scan_width = 240  # total pixels to scan
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
            clusters = []
            current_cluster = []
            for i in black_pixels:
                if not current_cluster:
                    current_cluster.append(i)
                else:
                    if i - current_cluster[-1] <= 5:
                        current_cluster.append(i)
                    else:
                        clusters.append(current_cluster)
                        current_cluster = [i]
            if current_cluster:
                clusters.append(current_cluster)
                
            # Filter out thick clusters (which are actually crosslines encountered perpendicularly)
            # A valid lane line should ideally be just a few pixels wide (e.g. BORDER_WIDTH=3, so maybe up to 10-15)
            valid_clusters = [cl for cl in clusters if len(cl) <= 20]
            
            if len(valid_clusters) >= 2:
                # Two lines detected: we want the center between the furthest left and right clusters
                c1 = sum(valid_clusters[0]) / len(valid_clusters[0])
                c2 = sum(valid_clusters[-1]) / len(valid_clusters[-1])
                error = (c1 + c2) / 2.0
            else:
                # If only 1 line or 0 lines are detected, we cannot safely determine lane center
                # without risking a positive feedback yeet. Trust the blind drive momentum.
                error = None
            
            # Find actual center point for drawing
            if error is not None:
                center_pt = (int(cx + error * dx), int(cy + error * dy))
            
        return error, scan_points, center_pt, detection_region

def draw_track(surface):
    surface.fill(WHITE)
    # The outer boundary of the entire track.
    pygame.draw.rect(surface, BLACK, (OUTER_LEFT, OUTER_TOP, OUTER_WIDTH, OUTER_HEIGHT), width=BORDER_WIDTH)
    
    # We replace the solid black lines with two visual boundaries defining the lanes.
    # The lane width is strictly LANE_WIDTH_PX. 
    # We slice the inner area into 4 small islands to create the crossroad in the middle.
    island_w = (INNER_WIDTH - LANE_WIDTH_PX) // 2 
    island_h = (INNER_HEIGHT - LANE_WIDTH_PX) // 2
    
    # Top-Left island
    pygame.draw.rect(surface, BLACK, (INNER_LEFT, INNER_TOP, island_w, island_h), width=BORDER_WIDTH)
    # Top-Right island
    pygame.draw.rect(surface, BLACK, (INNER_LEFT + island_w + LANE_WIDTH_PX, INNER_TOP, island_w, island_h), width=BORDER_WIDTH)
    # Bottom-Left island
    pygame.draw.rect(surface, BLACK, (INNER_LEFT, INNER_TOP + island_h + LANE_WIDTH_PX, island_w, island_h), width=BORDER_WIDTH)
    # Bottom-Right island
    pygame.draw.rect(surface, BLACK, (INNER_LEFT + island_w + LANE_WIDTH_PX, INNER_TOP + island_h + LANE_WIDTH_PX, island_w, island_h), width=BORDER_WIDTH)
    
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
    spawn_x, spawn_y = random_spawn_on_track(track_surface)
    car = SimulatedCar(x=spawn_x, y=spawn_y)
    car.angle = heading_to_angle(DEFAULT_HEADING)

    brain = None
    selected_start_screen = None
    selected_targets_screen = []
    selected_targets_brain = []
    mission_started = not WAIT_FOR_4_POINTS
    hit_int = False
    last_safe_pos = (car.x, car.y)
    lost_line_frames = 0
    stuck_frames = 0

    if USE_NEW_BRAIN:
        brain = RobotBrain(
            map_spec=MapSpec(
                half_width=ROAD_WIDTH / 2.0,
                half_height=ROAD_HEIGHT / 2.0,
            )
        )
        canonical_start = screen_to_canonical_brain((car.x, car.y), brain.map_spec)
        if canonical_start is None:
            canonical_start = (0.0, 0.0)
        bx, by = canonical_start
        car.x, car.y = brain_to_screen(canonical_start)
        brain.pose = Pose(x=bx, y=by, heading=DEFAULT_HEADING)
        brain.state = BrainState.STATE_PLANNING

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
                if event.key == pygame.K_n:
                    car.angle = heading_to_angle(Heading.N)
                    if USE_NEW_BRAIN:
                        brain.pose.heading = Heading.N
                if event.key == pygame.K_s:
                    car.angle = heading_to_angle(Heading.S)
                    if USE_NEW_BRAIN:
                        brain.pose.heading = Heading.S
                if event.key == pygame.K_e:
                    car.angle = heading_to_angle(Heading.E)
                    if USE_NEW_BRAIN:
                        brain.pose.heading = Heading.E
                if event.key == pygame.K_w:
                    car.angle = heading_to_angle(Heading.W)
                    if USE_NEW_BRAIN:
                        brain.pose.heading = Heading.W
                if event.key == pygame.K_r:
                    selected_start_screen = None
                    selected_targets_screen.clear()
                    selected_targets_brain.clear()
                    mission_started = not WAIT_FOR_4_POINTS
                    if USE_NEW_BRAIN:
                        canonical_start = screen_to_canonical_brain((car.x, car.y), brain.map_spec)
                        if canonical_start is None:
                            canonical_start = (0.0, 0.0)
                        bx, by = canonical_start
                        car.x, car.y = brain_to_screen(canonical_start)
                        brain.pose = Pose(x=bx, y=by, heading=brain.pose.heading)
                        brain.route_plan = None
                        brain.action_queue.clear()
                        brain.current_action = None
                        brain.state = BrainState.STATE_PLANNING
                if event.key == pygame.K_SPACE:
                    rx, ry = random_spawn_on_track(track_surface)
                    car.x, car.y = rx, ry
                    selected_start_screen = (rx, ry)
                    if USE_NEW_BRAIN:
                        canonical_start = screen_to_canonical_brain((car.x, car.y), brain.map_spec)
                        if canonical_start is None:
                            canonical_start = (0.0, 0.0)
                        bx, by = canonical_start
                        car.x, car.y = brain_to_screen(canonical_start)
                        selected_start_screen = (car.x, car.y)
                        brain.pose = Pose(x=bx, y=by, heading=brain.pose.heading)
                        brain.route_plan = None
                        brain.action_queue.clear()
                        brain.current_action = None
                        brain.state = BrainState.STATE_PLANNING
                if WAIT_FOR_4_POINTS and event.key == pygame.K_RETURN and len(selected_targets_screen) == 4 and selected_start_screen is not None:
                    mission_started = True
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
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                if 0 <= mx < MAP_WIDTH and 0 <= my < HEIGHT and len(selected_targets_screen) < 4:
                    snapped = find_nearest_track_point(track_surface, mx, my)
                    if snapped is not None:
                        candidate_brain = screen_to_brain(snapped)
                        if USE_NEW_BRAIN:
                            candidate_brain = snap_brain_to_canonical_path(
                                candidate_brain,
                                brain.map_spec.half_width,
                                brain.map_spec.half_height,
                                tol=LANE_WIDTH,
                            )
                            if candidate_brain is None:
                                continue
                            snapped = brain_to_screen(candidate_brain)
                        if (not USE_NEW_BRAIN) or brain.map_spec.is_on_valid_path(candidate_brain):
                            # Avoid accidental duplicate targets caused by repeated clicks around the same node.
                            if any(abs(candidate_brain[0] - t[0]) < 1.0 and abs(candidate_brain[1] - t[1]) < 1.0 for t in selected_targets_brain):
                                continue
                            selected_targets_screen.append(snapped)
                            selected_targets_brain.append(candidate_brain)
                            if len(selected_targets_screen) == 4 and WAIT_FOR_4_POINTS:
                                print("[Simulator] 4 targets selected. Press ENTER to start mission.")
                                
                            if len(selected_targets_screen) == 4 and not WAIT_FOR_4_POINTS:
                                mission_started = True
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
                mx, my = event.pos
                if 0 <= mx < MAP_WIDTH and 0 <= my < HEIGHT:
                    snapped = find_nearest_track_point(track_surface, mx, my)
                    if snapped is not None:
                        selected_start_screen = snapped
                        car.x, car.y = selected_start_screen
                        mission_started = False
                        if USE_NEW_BRAIN:
                            canonical_start = screen_to_canonical_brain(selected_start_screen, brain.map_spec)
                            if canonical_start is None:
                                continue
                            bx, by = canonical_start
                            car.x, car.y = brain_to_screen(canonical_start)
                            selected_start_screen = (car.x, car.y)
                            brain.pose = Pose(x=bx, y=by, heading=brain.pose.heading)
                            brain.route_plan = None
                            brain.action_queue.clear()
                            brain.current_action = None
                            brain.state = BrainState.STATE_PLANNING
                
        # --- SENSOR LOGIC (VIRTUAL CAMERA) ---
        # Updated signature: get_camera_error now returns 4 values
        error, scan_points, center_pt, detection_region = car.get_camera_error(track_surface, get_pixel_color)
        
        # --- CORE BACKEND LOGIC ---
        if USE_NEW_BRAIN:
            dt = 1.0 / max(target_fps, 1)

            if WAIT_FOR_4_POINTS and not mission_started:
                pid_output = 0.0
                p_term = 0.0
                d_term = 0.0
                state = "WAITING_TARGETS"
                reasoning = (
                    f"Start: {'SET' if selected_start_screen else 'NOT SET'} | "
                    f"Targets: {len(selected_targets_screen)}/4 | Press ENTER to run"
                )
            else:
                if brain.route_plan is None and len(selected_targets_brain) == 4:
                    brain.state = BrainState.STATE_PLANNING

                current_brain_raw = screen_to_brain((car.x, car.y))
                current_brain_pos = snap_brain_to_canonical_path(
                    current_brain_raw,
                    brain.map_spec.half_width,
                    brain.map_spec.half_height,
                    tol=40.0,
                )
                if current_brain_pos is None:
                    current_brain_pos = current_brain_raw
                
                # Simulate down-facing crossline detector: True if physically near any topological intersection
                hit_int = False
                for node in brain.map_spec.base_nodes():
                    dx = abs(current_brain_pos[0] - node[0])
                    dy = abs(current_brain_pos[1] - node[1])
                    # The PID might cause the car to be 10-20 pixels off-center laterally.
                    # We use a bounding box check (25px) to ensure we don't miss the node while crossing it.
                    if dx <= 25.0 and dy <= 25.0:
                        hit_int = True
                        break

                sensor_obs = SensorObservation(
                    current_position=current_brain_pos,
                    line_error=error,
                    camera_error=error,
                    hit_intersection=hit_int,
                    targets=selected_targets_brain if brain.route_plan is None and len(selected_targets_brain) == 4 else None,
                )
                cmd = brain.step(sensor_obs, dt)
                is_turning_action = (
                    brain.current_action is not None
                    and brain.current_action.action_type in {
                        ActionType.TURN_LEFT_90,
                        ActionType.TURN_RIGHT_90,
                        ActionType.TURN_180,
                    }
                )

                # Actuator mapping: differential command -> forward and turning motion in Pygame.
                forward = (cmd.left_speed - cmd.right_speed) * 0.5
                turn = (cmd.left_speed + cmd.right_speed) * 0.5

                speed_px_per_sec = 120.0
                turn_deg_per_sec = 140.0
                heading_lock_deg_per_sec = 220.0

                prev_x, prev_y = car.x, car.y
                car.angle -= turn * turn_deg_per_sec * dt

                # Keep body heading close to topological heading while driving straight.
                desired_angle = heading_to_angle(brain.pose.heading)
                if abs(turn) < 0.08:
                    diff = (desired_angle - car.angle + 180.0) % 360.0 - 180.0
                    max_step = heading_lock_deg_per_sec * dt
                    if diff > max_step:
                        diff = max_step
                    elif diff < -max_step:
                        diff = -max_step
                    car.angle += diff

                rad = math.radians(car.angle)
                car.x += forward * speed_px_per_sec * math.cos(rad) * dt
                car.y -= forward * speed_px_per_sec * math.sin(rad) * dt

                # Guard against ramming outer corners with a wrong body angle.
                if abs(forward) > 0.2:
                    sign = 1.0 if forward >= 0.0 else -1.0
                    dir_x = math.cos(rad) * sign
                    dir_y = -math.sin(rad) * sign
                    near_left = car.x <= OUTER_LEFT + 6
                    near_right = car.x >= OUTER_LEFT + OUTER_WIDTH - 6
                    near_top = car.y <= OUTER_TOP + 6
                    near_bottom = car.y >= OUTER_TOP + OUTER_HEIGHT - 6
                    pushing_out = (
                        (near_left and dir_x < -0.15)
                        or (near_right and dir_x > 0.15)
                        or (near_top and dir_y < -0.15)
                        or (near_bottom and dir_y > 0.15)
                    )
                    if pushing_out:
                        car.x, car.y = last_safe_pos
                        car.angle = desired_angle
                        lost_line_frames = 0
                        stuck_frames = 0

                # Failsafe: if we drift outside canonical corridor and camera also loses track,
                # snap back to the last safe pose to prevent runaway off-map behavior.
                post_brain = screen_to_brain((car.x, car.y))
                post_snap = snap_brain_to_canonical_path(
                    post_brain,
                    brain.map_spec.half_width,
                    brain.map_spec.half_height,
                    tol=40.0,
                )
                nearest_proj, proj_dist = project_brain_to_canonical_path(
                    post_brain,
                    brain.map_spec.half_width,
                    brain.map_spec.half_height,
                )

                if error is None:
                    lost_line_frames += 1
                else:
                    lost_line_frames = 0

                moved = math.hypot(car.x - prev_x, car.y - prev_y)
                if abs(forward) > 0.2 and moved < 0.2:
                    stuck_frames += 1
                else:
                    stuck_frames = 0

                # Corridor clamp: if drift gets too large, bring the car back to the nearest valid path.
                clamp_limit = TURN_CORRIDOR_CLAMP_PX if (is_turning_action or error is None) else CORRIDOR_CLAMP_PX
                if nearest_proj is not None and proj_dist > clamp_limit:
                    car.x, car.y = brain_to_screen(nearest_proj)
                    post_brain = screen_to_brain((car.x, car.y))
                    post_snap = snap_brain_to_canonical_path(
                        post_brain,
                        brain.map_spec.half_width,
                        brain.map_spec.half_height,
                        tol=40.0,
                    )

                if post_snap is None and error is None:
                    if nearest_proj is not None:
                        car.x, car.y = brain_to_screen(nearest_proj)
                    else:
                        car.x, car.y = last_safe_pos
                else:
                    last_safe_pos = (car.x, car.y)

                # Recovery mode: if line is lost too long or movement stalls, snap & realign.
                # While actively turning, allow longer line loss and avoid angle reset that can cancel the turn.
                line_loss_limit = 90 if is_turning_action else 20
                if lost_line_frames > line_loss_limit or stuck_frames > 10:
                    if nearest_proj is not None:
                        car.x, car.y = brain_to_screen(nearest_proj)
                    else:
                        car.x, car.y = last_safe_pos
                    if not is_turning_action:
                        car.angle = heading_to_angle(brain.pose.heading)
                    last_safe_pos = (car.x, car.y)
                    lost_line_frames = 0
                    stuck_frames = 0

                # Boundary collision guard using actual map geometry.
                # If car body center lands on dark border/island line, bounce back to last safe point.
                center_color = get_pixel_color(track_surface, int(car.x), int(car.y))
                if center_color[0] < TRACK_DARK_THRESHOLD:
                    car.x, car.y = last_safe_pos
                    if not is_turning_action:
                        car.angle = heading_to_angle(brain.pose.heading)

                # Hard boundary guard for runaway edge cases.
                if error is None:
                    out_of_track_bounds = (
                        car.x < OUTER_LEFT - EDGE_MARGIN_PX
                        or car.x > OUTER_LEFT + OUTER_WIDTH + EDGE_MARGIN_PX
                        or car.y < OUTER_TOP - EDGE_MARGIN_PX
                        or car.y > OUTER_TOP + OUTER_HEIGHT + EDGE_MARGIN_PX
                    )
                    if out_of_track_bounds:
                        car.x, car.y = last_safe_pos

                # Keep simulated robot on visible map.
                car.x = max(0.0, min(float(MAP_WIDTH - 1), car.x))
                car.y = max(0.0, min(float(HEIGHT - 1), car.y))

                pid_output = turn
                p_term = 0.0
                d_term = 0.0
                state = brain.state.name
                reasoning = f"Brain cmd L={cmd.left_speed:.2f} R={cmd.right_speed:.2f} done={cmd.done}"
        else:
            pid_output, reasoning, p_term, d_term = logic.calculate_pid(error)
            state = logic.update_state(horizontal_line_detected=False, trigger_zone_reached=False) # Simplified for basic following
            logic.log_data(frame_count, error, p_term, d_term, pid_output, reasoning)
            
            # --- MOVE CAR ---
            car.move(pid_output)
        
        # --- RENDER MAP ---
        screen.blit(track_surface, (0,0))

        # Draw user-selected floating targets (must pick 4 before mission starts).
        for idx, pt in enumerate(selected_targets_screen):
            pygame.draw.circle(screen, (255, 165, 0), (int(pt[0]), int(pt[1])), 6)
            label = small_font.render(str(idx + 1), True, BLACK)
            screen.blit(label, (int(pt[0]) + 8, int(pt[1]) - 8))

        # Draw chosen start location marker.
        if selected_start_screen is not None:
            sx, sy = int(selected_start_screen[0]), int(selected_start_screen[1])
            pygame.draw.circle(screen, (0, 120, 255), (sx, sy), 9, 2)
            pygame.draw.line(screen, (0, 120, 255), (sx - 6, sy), (sx + 6, sy), 2)
            pygame.draw.line(screen, (0, 120, 255), (sx, sy - 6), (sx, sy + 6), 2)
            s_label = small_font.render("START", True, (0, 120, 255))
            screen.blit(s_label, (sx + 10, sy + 6))

        # Draw planned path from new brain when available.
        if USE_NEW_BRAIN and brain is not None and brain.route_plan is not None and len(brain.route_plan.polyline) > 1:
            path_screen = [brain_to_screen(p) for p in brain.route_plan.polyline]
            pygame.draw.lines(screen, (255, 140, 0), False, path_screen, 2)

        # Compact in-canvas interaction hints for quick onboarding.
        hint_lines = [
            "Right click on BLACK line: set START",
            "Left click on BLACK line: add TARGET (x4)",
            "ENTER run | N/S/E/W heading | SPACE random start | R reset",
        ]
        hint_bg = pygame.Surface((MAP_WIDTH, 54), pygame.SRCALPHA)
        hint_bg.fill((20, 20, 20, 170))
        screen.blit(hint_bg, (0, 0))
        for idx, line in enumerate(hint_lines):
            hint_surface = small_font.render(line, True, WHITE)
            screen.blit(hint_surface, (10, 5 + idx * 16))

        # Setup checklist panel.
        checklist_bg = pygame.Surface((250, 78), pygame.SRCALPHA)
        checklist_bg.fill((10, 10, 10, 170))
        screen.blit(checklist_bg, (10, 58))
        checklist = [
            ("Start selected", selected_start_screen is not None),
            ("4 targets selected", len(selected_targets_screen) == 4),
            ("Mission started", mission_started),
        ]
        for idx, (label, ok) in enumerate(checklist):
            color = GREEN if ok else RED
            text = small_font.render(("[OK] " if ok else "[  ] ") + label, True, color)
            screen.blit(text, (16, 64 + idx * 22))
        
        # ========== DETECTION VISUALIZATION (RED AREAS) ==========
        # Draw detection region in RED with transparency
        if len(detection_region) > 2:
            pygame.draw.circle(screen, RED, center_pt if center_pt else (0,0), 25)
            # Draw filled detection area
            pygame.draw.lines(screen, DARK_RED, False, detection_region, 4)
        
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
                f"Brain: {'NEW' if USE_NEW_BRAIN else 'LEGACY'}",
                f"Start: {'SET' if selected_start_screen else 'NOT SET'}",
                f"Targets: {len(selected_targets_screen)}/4",
                f"Mission: {'RUN' if mission_started else 'WAIT'}",
                f"Heading Set: {brain.pose.heading.value if USE_NEW_BRAIN and brain is not None else 'LEGACY'}",
                f"State: {state}",
                f"Detection: {'✓ YES' if center_pt else '✗ LOST'}",
                f"Crossline: {'✓ HIT' if USE_NEW_BRAIN and hit_int else '---'}",
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
    main(slow_motion_factor=1, debug_verbose=True)
