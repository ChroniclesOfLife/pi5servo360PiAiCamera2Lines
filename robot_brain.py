from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
import heapq
import itertools
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


Point = Tuple[float, float]


class Heading(str, Enum):
    N = "N"
    E = "E"
    S = "S"
    W = "W"


class BrainState(Enum):
    STATE_KIDNAPPED = auto()
    STATE_PLANNING = auto()
    STATE_EXECUTING = auto()


class ActionType(str, Enum):
    DRIVE_TO_INTERSECTION = "DRIVE_TO_INTERSECTION"
    DRIVE_TO_FLOATING_POINT = "DRIVE_TO_FLOATING_POINT"
    TURN_LEFT_90 = "TURN_LEFT_90"
    TURN_RIGHT_90 = "TURN_RIGHT_90"
    TURN_180 = "TURN_180"


@dataclass
class Pose:
    x: float
    y: float
    heading: Heading


@dataclass
class Action:
    action_type: ActionType
    target: Optional[Point] = None
    note: str = ""


@dataclass
class LocalizationObservation:
    can_turn_right: bool
    can_go_straight: bool
    can_turn_left: bool


@dataclass
class LocalizationResult:
    localized: bool
    pose: Pose
    note: str


@dataclass
class RoutePlan:
    ordered_targets: List[Point]
    total_distance: float
    polyline: List[Point]


@dataclass
class MotorCommand:
    left_speed: float
    right_speed: float
    done: bool = False


@dataclass
class SensorObservation:
    current_position: Point
    line_error: Optional[float] = None
    camera_error: Optional[float] = None
    localization: Optional[LocalizationObservation] = None
    targets: Optional[Sequence[Point]] = None
    hit_intersection: bool = False
    odometry_ticks: int = 0


@dataclass
class MapSpec:
    half_width: float = 240.0
    half_height: float = 160.0

    def _eps(self) -> float:
        return 1e-6

    def base_nodes(self) -> List[Point]:
        hw = self.half_width
        hh = self.half_height
        return [
            (-hw, -hh),
            (-hw, hh),
            (hw, -hh),
            (hw, hh),
            (0.0, 0.0),
            (0.0, -hh),
            (0.0, hh),
            (-hw, 0.0),
            (hw, 0.0),
        ]

    def is_on_valid_path(self, p: Point) -> bool:
        x, y = p
        hw = self.half_width
        hh = self.half_height
        e = self._eps()

        on_top = abs(y - hh) <= e and -hw - e <= x <= hw + e
        on_bottom = abs(y + hh) <= e and -hw - e <= x <= hw + e
        on_left = abs(x + hw) <= e and -hh - e <= y <= hh + e
        on_right = abs(x - hw) <= e and -hh - e <= y <= hh + e
        on_vmid = abs(x) <= e and -hh - e <= y <= hh + e
        on_hmid = abs(y) <= e and -hw - e <= x <= hw + e
        return on_top or on_bottom or on_left or on_right or on_vmid or on_hmid

    def is_intersection(self, p: Point) -> bool:
        x, y = p
        hw = self.half_width
        hh = self.half_height
        e = self._eps()
        cond_center = abs(x) <= e and abs(y) <= e
        cond_top_mid = abs(x) <= e and abs(y - hh) <= e
        cond_bottom_mid = abs(x) <= e and abs(y + hh) <= e
        cond_left_mid = abs(x + hw) <= e and abs(y) <= e
        cond_right_mid = abs(x - hw) <= e and abs(y) <= e
        cond_corners = (abs(x + hw) <= e or abs(x - hw) <= e) and (abs(y + hh) <= e or abs(y - hh) <= e)
        return cond_center or cond_top_mid or cond_bottom_mid or cond_left_mid or cond_right_mid or cond_corners

    def segment_definitions(self) -> List[Tuple[str, float, float, float]]:
        hw = self.half_width
        hh = self.half_height
        # format: (axis, fixed_value, min, max)
        return [
            ("h", hh, -hw, hw),
            ("h", -hh, -hw, hw),
            ("h", 0.0, -hw, hw),
            ("v", -hw, -hh, hh),
            ("v", hw, -hh, hh),
            ("v", 0.0, -hh, hh),
        ]


@dataclass
class Navigator:
    map_spec: MapSpec

    def _distance(self, a: Point, b: Point) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _is_on_segment(self, p: Point, seg: Tuple[str, float, float, float]) -> bool:
        axis, fixed, low, high = seg
        x, y = p
        e = 1e-6
        if axis == "h":
            return abs(y - fixed) <= e and low - e <= x <= high + e
        return abs(x - fixed) <= e and low - e <= y <= high + e

    def _nodes_for_graph(self, extra_points: Iterable[Point]) -> List[Point]:
        nodes = list(self.map_spec.base_nodes())
        for p in extra_points:
            if not self.map_spec.is_on_valid_path(p):
                raise ValueError(f"Point {p} is not on a valid path")
            nodes.append((float(p[0]), float(p[1])))

        # Deduplicate with rounding for stable keying.
        seen = {}
        for x, y in nodes:
            key = (round(x, 6), round(y, 6))
            seen[key] = (float(x), float(y))
        return list(seen.values())

    def _build_graph(self, nodes: Sequence[Point]) -> Dict[Point, List[Tuple[Point, float]]]:
        graph: Dict[Point, List[Tuple[Point, float]]] = {n: [] for n in nodes}
        segments = self.map_spec.segment_definitions()

        for seg in segments:
            axis, _, _, _ = seg
            seg_nodes = [n for n in nodes if self._is_on_segment(n, seg)]
            if axis == "h":
                seg_nodes.sort(key=lambda p: p[0])
            else:
                seg_nodes.sort(key=lambda p: p[1])

            for i in range(len(seg_nodes) - 1):
                a = seg_nodes[i]
                b = seg_nodes[i + 1]
                w = self._distance(a, b)
                if w <= 0:
                    continue
                graph[a].append((b, w))
                graph[b].append((a, w))

        return graph

    def _dijkstra(self, graph: Dict[Point, List[Tuple[Point, float]]], start: Point) -> Tuple[Dict[Point, float], Dict[Point, Optional[Point]]]:
        dist: Dict[Point, float] = {node: math.inf for node in graph}
        prev: Dict[Point, Optional[Point]] = {node: None for node in graph}
        dist[start] = 0.0
        heap: List[Tuple[float, Point]] = [(0.0, start)]

        while heap:
            cur_dist, node = heapq.heappop(heap)
            if cur_dist > dist[node]:
                continue
            for nxt, w in graph[node]:
                nd = cur_dist + w
                if nd < dist[nxt]:
                    dist[nxt] = nd
                    prev[nxt] = node
                    heapq.heappush(heap, (nd, nxt))

        return dist, prev

    def shortest_path(self, start: Point, goal: Point, context_points: Optional[Iterable[Point]] = None) -> Tuple[float, List[Point]]:
        points = list(context_points or [])
        nodes = self._nodes_for_graph(points + [start, goal])
        graph = self._build_graph(nodes)
        dist, prev = self._dijkstra(graph, start)

        if dist[goal] == math.inf:
            raise RuntimeError(f"No path between {start} and {goal}")

        # Reconstruct path.
        path: List[Point] = []
        cur: Optional[Point] = goal
        while cur is not None:
            path.append(cur)
            cur = prev[cur]
        path.reverse()
        return dist[goal], path

    def plan_route(self, start: Pose, targets: Sequence[Point]) -> RoutePlan:
        if len(targets) != 4:
            raise ValueError("Exactly 4 targets are required")

        points = [tuple(t) for t in targets]
        for p in points:
            if not self.map_spec.is_on_valid_path(p):
                raise ValueError(f"Target {p} is outside valid paths")

        start_p = (start.x, start.y)
        all_points = [start_p] + points

        best_order: Optional[Tuple[Point, ...]] = None
        best_total = math.inf
        best_polyline: List[Point] = []

        for order in itertools.permutations(points):
            total = 0.0
            polyline: List[Point] = [start_p]
            cur = start_p
            valid = True

            for nxt in order:
                try:
                    segment_dist, segment_path = self.shortest_path(cur, nxt, context_points=all_points)
                except RuntimeError:
                    valid = False
                    break
                total += segment_dist
                if len(segment_path) > 1:
                    polyline.extend(segment_path[1:])
                cur = nxt

            if valid and total < best_total:
                best_total = total
                best_order = order
                best_polyline = polyline

        if best_order is None:
            raise RuntimeError("No valid TSP route found")

        return RoutePlan(ordered_targets=list(best_order), total_distance=best_total, polyline=best_polyline)

    def _segment_heading(self, a: Point, b: Point) -> Heading:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        if abs(dx) > abs(dy):
            return Heading.E if dx > 0 else Heading.W
        return Heading.N if dy > 0 else Heading.S

    def _turn_action(self, current: Heading, new: Heading) -> Optional[ActionType]:
        if current == new:
            return None

        order = [Heading.N, Heading.E, Heading.S, Heading.W]
        ci = order.index(current)
        ni = order.index(new)
        diff = (ni - ci) % 4
        if diff == 1:
            return ActionType.TURN_RIGHT_90
        if diff == 3:
            return ActionType.TURN_LEFT_90
        return ActionType.TURN_180

    def build_action_queue(self, start_pose: Pose, route_plan: RoutePlan) -> List[Action]:
        if len(route_plan.polyline) < 2:
            return []

        actions: List[Action] = []
        heading = start_pose.heading
        targets_set = set(route_plan.ordered_targets)

        for i in range(len(route_plan.polyline) - 1):
            a = route_plan.polyline[i]
            b = route_plan.polyline[i + 1]
            seg_heading = self._segment_heading(a, b)

            turn = self._turn_action(heading, seg_heading)
            if turn is not None:
                actions.append(Action(turn, note=f"Reorient {heading.value}->{seg_heading.value}"))
            heading = seg_heading

            if self.map_spec.is_intersection(b):
                actions.append(Action(ActionType.DRIVE_TO_INTERSECTION, target=b))
            elif b in targets_set:
                actions.append(Action(ActionType.DRIVE_TO_FLOATING_POINT, target=b))
            else:
                # Non-target node on line segment; keep command explicit for deterministic replay.
                actions.append(Action(ActionType.DRIVE_TO_FLOATING_POINT, target=b, note="Transit point"))

        return actions


@dataclass
class MotionController:
    kp: float = 0.01
    kd: float = 0.002
    max_speed: float = 0.6
    min_speed: float = 0.12
    brake_distance: float = 40.0
    turn_speed: float = 0.35
    align_error_threshold: float = 3.0
    prev_error: float = 0.0
    current_turn_time: float = 0.0
    last_valid_error: float = 0.0
    line_lost_time: float = 0.0
    line_hold_time: float = 0.35

    def reset(self) -> None:
        self.prev_error = 0.0
        self.current_turn_time = 0.0
        self.last_valid_error = 0.0
        self.line_lost_time = 0.0

    def resolved_line_error(self, error: Optional[float], dt: float) -> Optional[float]:
        if error is not None:
            self.last_valid_error = error
            self.line_lost_time = 0.0
            return error

        self.line_lost_time += max(0.0, dt)
        if self.line_lost_time <= self.line_hold_time:
            decay = max(0.0, 1.0 - (self.line_lost_time / max(self.line_hold_time, 1e-6)))
            return self.last_valid_error * decay
        return None

    def pid_steering(self, error: Optional[float]) -> float:
        if error is None:
            return 0.0
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.kd * derivative

    def speed_profile(self, distance_remaining: float, total_distance: float) -> float:
        # Trapezoidal-like profile with simple linear ramp in the last brake_distance window.
        if total_distance <= 1e-6:
            return self.min_speed

        cruise = min(self.max_speed, max(self.min_speed, self.max_speed))
        if distance_remaining >= self.brake_distance:
            return cruise

        ratio = max(0.0, min(1.0, distance_remaining / max(self.brake_distance, 1e-6)))
        return self.min_speed + (cruise - self.min_speed) * ratio

    def drive_command_blind(self, error: Optional[float], hit_intersection: bool, dt: float) -> MotorCommand:
        if hit_intersection:
            return MotorCommand(0.0, 0.0, done=True)

        resolved_error = self.resolved_line_error(error, dt)

        if resolved_error is None:
            # Reached an intersection gap or lost line, maintain forward momentum without steering
            return MotorCommand(self.max_speed, -self.max_speed, done=False)

        base = self.max_speed
        steer = self.pid_steering(resolved_error)
        steer = max(-base, min(base, steer))

        return MotorCommand(base + steer, -(base - steer), done=False)

    def drive_command_dead_reckoning(self, error: Optional[float], distance_remaining: float, total_distance: float, dt: float) -> MotorCommand:
        base = self.speed_profile(distance_remaining, total_distance)

        resolved_error = self.resolved_line_error(error, dt)

        if resolved_error is None:
            # Reached a gap or lost line, maintain forward momentum without steering
            steer = 0.0
        else:
            steer = self.pid_steering(resolved_error)
            steer = max(-base, min(base, steer))

        left = base + steer
        right = -(base - steer)

        done = distance_remaining <= 15.0  # Increased tolerance to allow for lateral PID sway
        if done:
            return MotorCommand(0.0, 0.0, done=True)
        return MotorCommand(left, right, done=False)

    def active_turn_command(self, camera_error: Optional[float], turn_direction: ActionType, dt: float) -> MotorCommand:
        if turn_direction not in {ActionType.TURN_LEFT_90, ActionType.TURN_RIGHT_90, ActionType.TURN_180}:
            raise ValueError("turn_direction must be a TURN action")

        self.current_turn_time += dt
        
        # Open loop duration: wait until robot has turned enough off the old line
        # At turn_speed=0.35 and 140 deg/sec, 90 deg takes ~1.83s. We wait 1.4s.
        min_turn_time = 1.4 if turn_direction != ActionType.TURN_180 else 2.8
        max_turn_time = 2.6 if turn_direction != ActionType.TURN_180 else 5.2

        # Safety timeout: avoid infinite spinning when camera stays lost.
        if self.current_turn_time >= max_turn_time:
            self.current_turn_time = 0.0
            return MotorCommand(0.0, 0.0, done=True)

        # Closed-loop stop condition: when camera aligns to new line roughly.
        # We don't require perfect <= 3.0 error, just seeing the 2 lines of the new lane
        # after the minimum turn time is usually enough to hand over to PID.
        if self.current_turn_time > min_turn_time and camera_error is not None and abs(camera_error) <= 20.0:
            self.current_turn_time = 0.0  # reset for next turn
            return MotorCommand(0.0, 0.0, done=True)

        if turn_direction == ActionType.TURN_LEFT_90:
            return MotorCommand(-self.turn_speed, -self.turn_speed, done=False)
        if turn_direction == ActionType.TURN_RIGHT_90:
            return MotorCommand(self.turn_speed, self.turn_speed, done=False)

        # TURN_180
        return MotorCommand(self.turn_speed, self.turn_speed, done=False)

    def checkpoint_calibration(self, pose: Pose, checkpoint: Point) -> Pose:
        return Pose(x=checkpoint[0], y=checkpoint[1], heading=pose.heading)


@dataclass
class RobotBrain:
    map_spec: MapSpec = field(default_factory=MapSpec)
    navigator: Navigator = field(init=False)
    motion: MotionController = field(default_factory=MotionController)
    state: BrainState = BrainState.STATE_KIDNAPPED
    pose: Pose = field(default_factory=lambda: Pose(0.0, 0.0, Heading.N))
    action_queue: List[Action] = field(default_factory=list)
    route_plan: Optional[RoutePlan] = None
    current_action: Optional[Action] = None
    action_elapsed_s: float = 0.0
    action_last_remaining: Optional[float] = None
    action_stuck_frames: int = 0

    def __post_init__(self) -> None:
        self.navigator = Navigator(self.map_spec)

    def _turn_right(self, heading: Heading) -> Heading:
        return {
            Heading.N: Heading.E,
            Heading.E: Heading.S,
            Heading.S: Heading.W,
            Heading.W: Heading.N,
        }[heading]

    def _turn_left(self, heading: Heading) -> Heading:
        return {
            Heading.N: Heading.W,
            Heading.W: Heading.S,
            Heading.S: Heading.E,
            Heading.E: Heading.N,
        }[heading]

    def _corner_from_heading_and_turn(self, heading: Heading, only_turn: str) -> Point:
        hw = self.map_spec.half_width
        hh = self.map_spec.half_height

        table = {
            (Heading.N, "left"): (-hw, hh),
            (Heading.N, "right"): (hw, hh),
            (Heading.S, "left"): (hw, -hh),
            (Heading.S, "right"): (-hw, -hh),
            (Heading.E, "left"): (hw, hh),
            (Heading.E, "right"): (hw, -hh),
            (Heading.W, "left"): (-hw, -hh),
            (Heading.W, "right"): (-hw, hh),
        }
        return table[(heading, only_turn)]

    def localize(self, obs: LocalizationObservation) -> LocalizationResult:
        # Right-hand rule: right > straight > left, until corner where exactly one turn exists.
        open_count = int(obs.can_turn_right) + int(obs.can_go_straight) + int(obs.can_turn_left)
        if not obs.can_go_straight and open_count == 1:
            only_turn = "right" if obs.can_turn_right else "left"
            corner = self._corner_from_heading_and_turn(self.pose.heading, only_turn)
            self.pose = Pose(corner[0], corner[1], self.pose.heading)
            self.state = BrainState.STATE_PLANNING
            return LocalizationResult(True, self.pose, f"Localized at corner {corner} via dead-end rule")

        if obs.can_turn_right:
            self.pose.heading = self._turn_right(self.pose.heading)
            return LocalizationResult(False, self.pose, "Exploring with right-hand rule (turn right)")
        if obs.can_go_straight:
            return LocalizationResult(False, self.pose, "Exploring with right-hand rule (go straight)")
        if obs.can_turn_left:
            self.pose.heading = self._turn_left(self.pose.heading)
            return LocalizationResult(False, self.pose, "Exploring with right-hand rule (turn left)")

        # No exits: rotate 180 to continue exploration.
        self.pose.heading = self._turn_right(self._turn_right(self.pose.heading))
        return LocalizationResult(False, self.pose, "Dead stop encountered, reversing heading")

    def plan(self, targets: Sequence[Point]) -> RoutePlan:
        self.route_plan = self.navigator.plan_route(self.pose, targets)
        self.action_queue = self.navigator.build_action_queue(self.pose, self.route_plan)
        self.state = BrainState.STATE_EXECUTING
        return self.route_plan

    def pop_next_action(self) -> Optional[Action]:
        if not self.action_queue:
            return None
        return self.action_queue.pop(0)

    def execute_drive_step(self, action: Action, sensor_obs: SensorObservation, dt: float) -> MotorCommand:
        if action.target is None:
            return MotorCommand(0.0, 0.0, done=True)
            
        current_position = sensor_obs.current_position
        total = abs(action.target[0] - self.pose.x) + abs(action.target[1] - self.pose.y)
        rem = abs(action.target[0] - current_position[0]) + abs(action.target[1] - current_position[1])

        # Anti-stuck progress monitor.
        if self.action_last_remaining is None:
            self.action_last_remaining = rem
        else:
            progress = self.action_last_remaining - rem
            if progress > 0.5:
                self.action_stuck_frames = 0
            else:
                # Penalize only meaningful stagnation/regression; avoid false positives on small jitter.
                if rem > self.action_last_remaining + 0.8:
                    self.action_stuck_frames += 2
                elif sensor_obs.line_error is None and abs(progress) <= 0.15:
                    self.action_stuck_frames += 1
                else:
                    self.action_stuck_frames = max(0, self.action_stuck_frames - 1)
            self.action_last_remaining = rem

        soft_stuck = self.action_stuck_frames >= 120
        hard_timeout = self.action_elapsed_s >= 20.0
        
        if action.action_type == ActionType.DRIVE_TO_INTERSECTION:
            # Debounce early intersection signal right after action start.
            valid_hit = sensor_obs.hit_intersection and self.action_elapsed_s >= 0.20

            if rem <= 8.0 or hard_timeout:
                self.pose = self.motion.checkpoint_calibration(self.pose, action.target)
                return MotorCommand(0.0, 0.0, done=True)
            
            cmd = self.motion.drive_command_blind(sensor_obs.line_error, hit_intersection=valid_hit, dt=dt)
            if soft_stuck and sensor_obs.line_error is None and not cmd.done:
                cmd = MotorCommand(self.motion.min_speed, -self.motion.min_speed, done=False)
            if cmd.done:
                self.pose = self.motion.checkpoint_calibration(self.pose, action.target)
        else: # DRIVE_TO_FLOATING_POINT
            if rem <= 12.0 or hard_timeout:
                self.pose = Pose(action.target[0], action.target[1], self.pose.heading)
                return MotorCommand(0.0, 0.0, done=True)

            cmd = self.motion.drive_command_dead_reckoning(
                sensor_obs.line_error,
                distance_remaining=rem,
                total_distance=total,
                dt=dt,
            )
            if soft_stuck and sensor_obs.line_error is None and not cmd.done:
                cmd = MotorCommand(self.motion.min_speed, -self.motion.min_speed, done=False)
            if cmd.done:
                self.pose = Pose(action.target[0], action.target[1], self.pose.heading)

        return cmd

    def execute_turn_step(self, action: Action, camera_error: Optional[float], dt: float) -> MotorCommand:
        cmd = self.motion.active_turn_command(camera_error, action.action_type, dt)
        if cmd.done:
            if action.action_type == ActionType.TURN_LEFT_90:
                self.pose.heading = self._turn_left(self.pose.heading)
            elif action.action_type == ActionType.TURN_RIGHT_90:
                self.pose.heading = self._turn_right(self.pose.heading)
            elif action.action_type == ActionType.TURN_180:
                self.pose.heading = self._turn_right(self._turn_right(self.pose.heading))
        return cmd

    def step(self, sensor_obs: SensorObservation, dt: float) -> MotorCommand:
        if self.state == BrainState.STATE_KIDNAPPED:
            if sensor_obs.localization is None:
                return MotorCommand(0.0, 0.0, done=False)
            self.localize(sensor_obs.localization)
            return MotorCommand(0.0, 0.0, done=False)

        if self.state == BrainState.STATE_PLANNING:
            if sensor_obs.targets and self.route_plan is None:
                self.plan(sensor_obs.targets)
            if self.state != BrainState.STATE_EXECUTING:
                return MotorCommand(0.0, 0.0, done=False)

        if self.state != BrainState.STATE_EXECUTING:
            return MotorCommand(0.0, 0.0, done=False)

        if self.current_action is None:
            self.current_action = self.pop_next_action()
            if self.current_action is None:
                return MotorCommand(0.0, 0.0, done=True)
            self.action_elapsed_s = 0.0
            self.action_last_remaining = None
            self.action_stuck_frames = 0

        self.action_elapsed_s += dt

        action = self.current_action
        if action.action_type in {ActionType.TURN_LEFT_90, ActionType.TURN_RIGHT_90, ActionType.TURN_180}:
            cmd = self.execute_turn_step(action, sensor_obs.camera_error, dt)
        else:
            cmd = self.execute_drive_step(action, sensor_obs, dt)

        if cmd.done:
            self.current_action = None
            self.action_elapsed_s = 0.0
            self.action_last_remaining = None
            self.action_stuck_frames = 0

        return cmd
