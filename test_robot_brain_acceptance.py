from robot_brain import (
    Action,
    ActionType,
    BrainState,
    Heading,
    LocalizationObservation,
    MapSpec,
    MotionController,
    Pose,
    RobotBrain,
    SensorObservation,
)


def run_acceptance_tests() -> None:
    # 1) right-hand-rule corner localization
    brain = RobotBrain()
    brain.pose = Pose(0.0, 0.0, Heading.N)
    result = brain.localize(LocalizationObservation(can_turn_right=True, can_go_straight=False, can_turn_left=False))
    assert result.localized is True

    # 2) heading consistency after localization
    assert result.pose.heading == Heading.N
    assert result.pose.x == brain.map_spec.half_width and result.pose.y == brain.map_spec.half_height

    # 3) TSP route correctness on 4 points
    brain = RobotBrain()
    brain.pose = Pose(0.0, 0.0, Heading.E)
    targets = [(0.0, 200.0), (300.0, 0.0), (-300.0, 0.0), (0.0, -200.0)]
    plan = brain.plan(targets)
    assert len(plan.ordered_targets) == 4
    assert plan.total_distance > 0.0

    # 4) action queue generation validity
    assert len(brain.action_queue) > 0
    assert all(isinstance(a, Action) for a in brain.action_queue)

    # 5) checkpoint calibration update
    mc = MotionController()
    p = Pose(10.0, 20.0, Heading.W)
    snapped = mc.checkpoint_calibration(p, (0.0, 0.0))
    assert snapped.x == 0.0 and snapped.y == 0.0 and snapped.heading == Heading.W

    # 6) active visual turning convergence criterion
    turn_cmd = mc.active_turn_command(camera_error=0.4, turn_direction=ActionType.TURN_RIGHT_90)
    assert turn_cmd.done is True

    # 7) trapezoidal speed profile decreases near target
    far = mc.speed_profile(distance_remaining=200.0, total_distance=300.0)
    near = mc.speed_profile(distance_remaining=5.0, total_distance=300.0)
    assert near < far

    # 8) safe behavior when line is temporarily lost
    safe = mc.drive_command(error=None, distance_remaining=50.0, total_distance=100.0)
    assert safe.left_speed == 0.0 and safe.right_speed == 0.0 and safe.done is False

    # Extra smoke: step() contract transitions planning -> executing and yields command object.
    brain = RobotBrain()
    brain.state = BrainState.STATE_PLANNING
    obs = SensorObservation(
        current_position=(0.0, 0.0),
        line_error=0.0,
        camera_error=0.0,
        targets=targets,
    )
    cmd = brain.step(obs, dt=1.0 / 30.0)
    assert hasattr(cmd, "left_speed") and hasattr(cmd, "right_speed")


if __name__ == "__main__":
    run_acceptance_tests()
    print("All acceptance tests passed.")
