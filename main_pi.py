from motor_control import RobotMotors
from vision import LineFollowerVision
from robot_logic import RobotLogic
import time

def main(debug_mode=True, verbose_period=10):
    """
    Main entry point for physical robot execution on Raspberry Pi 5.
    
    Args:
        debug_mode (bool): Enable detailed console logging
        verbose_period (int): Print status every N frames
    """
    # Initialize hardware
    motors = RobotMotors(left_pin=12, right_pin=13)
    vision = LineFollowerVision(
        resolution=(640, 480),
        debug_mode=debug_mode,
        show_stages=False,
    )
    
    # Initialize Shared Logic Brain
    logic = RobotLogic(kp=0.005, kd=0.001, log_file="log_pibot.txt")
    
    # Setup Base Speed for MG996R 360 (0.0 to 1.0)
    BASE_SPEED = 0.3 
    
    if debug_mode:
        print("="*60)
        print("ROBOT LINE FOLLOWER - Raspberry Pi 5 with IMX500 AI Camera")
        print("="*60)
        print(f"[Config] BASE_SPEED: {BASE_SPEED}")
        print(f"[Config] Kp: {logic.kp}, Kd: {logic.kd}")
        print(f"[Config] Left Servo: GPIO 12, Right Servo: GPIO 13")
        print(f"[Status] Hardware initialized and ready")
    
    print("Robot Starting in 3 seconds... Digital Twin Logic Engaged.")
    time.sleep(3)
    frame_count = 0
    errors_lost = 0
    
    try:
        while True:
            # 1. Look for the line using camera
            error = vision.get_line_error()
            
            # --- CORE BACKEND LOGIC ---
            pid_output, reasoning, p_term, d_term = logic.calculate_pid(error)
            state = logic.update_state(horizontal_line_detected=False, trigger_zone_reached=False)
            logic.log_data(frame_count, error, p_term, d_term, pid_output, reasoning)
            
            # Track line loss frequency
            if error is None:
                errors_lost += 1
            
            # --- PRINT DETAILED LOGIC TO SSH CONSOLE ---
            if frame_count % verbose_period == 0:
                 err_str = f"{error:.1f}" if error is not None else "LOST"
                 print(f"[{frame_count:5d}] State: {state:10s} | Error: {err_str:>6s} | " + 
                       f"P: {p_term:7.3f} | D: {d_term:7.3f} | PID: {pid_output:7.3f}")
                 print(f"        => {reasoning}")
            
            # 2. Translate logic into physical movement
            if state == "FOLLOWING":
                if error is not None:
                    # Apply PID constraint for hardware safety
                    # Limits the turn aggressiveness to prevent spinning out
                    turn_adjust = max(-BASE_SPEED, min(BASE_SPEED, pid_output))
                    
                    left_speed = BASE_SPEED + turn_adjust
                    right_speed = -(BASE_SPEED - turn_adjust) # Inverted mounting
                    
                    motors.move(left_speed, right_speed)
                else:
                    if frame_count % verbose_period == 0:
                        print(f"[{frame_count}] Line Lost! Stopping for safety...")
                    motors.stop()
                    
            elif state == "TURNING_90":
                # Spin in place — state machine in robot_logic auto-exits
                # after turn_duration frames (default 30 ≈ 1 second)
                if frame_count % verbose_period == 0:
                     print(f"[{frame_count}] Executing 90 deg turn ({logic.turn_counter}/{logic.turn_duration})...")
                motors.move(0.5, 0.5) # Spin in place
            
            # Hardware loop delay matches camera framerate (~30fps)
            time.sleep(0.03)
            frame_count += 1
            
    except KeyboardInterrupt:
        print("\n" + "="*60)
        print("ROBOT STOPPED BY USER")
        print("="*60)
    finally:
        motors.stop()
        vision.stop()
        
        # Print summary statistics
        if debug_mode:
            print("\n" + "="*60)
            print("EXECUTION SUMMARY")
            print("="*60)
            print(f"Total Frames: {frame_count}")
            print(f"Line Detection Loss Events: {errors_lost} ({100*errors_lost/max(frame_count,1):.1f}%)")
            print(f"Log File: log_pibot.txt")
            print("Hardware Cleaned Up.")
            print("="*60)

if __name__ == "__main__":
    main(debug_mode=True, verbose_period=10)
