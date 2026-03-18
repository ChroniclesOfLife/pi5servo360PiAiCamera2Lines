from motor_control import RobotMotors
from vision import LineFollowerVision
from robot_logic import RobotLogic
import time

def main():
    # Initialize hardware
    motors = RobotMotors(left_pin=12, right_pin=13)
    vision = LineFollowerVision(resolution=(640, 480))
    
    # Initialize Shared Logic Brain
    logic = RobotLogic(kp=0.005, kd=0.001, log_file="log_pibot.txt")
    
    # Setup Base Speed for MG996R 360 (0.0 to 1.0)
    BASE_SPEED = 0.3 
    
    print("Robot Starting in 3 seconds... Digital Twin Logic Engaged.")
    time.sleep(3)
    frame_count = 0
    
    try:
        while True:
            # 1. Look for the line using camera
            error = vision.get_line_error()
            
            # --- CORE BACKEND LOGIC ---
            pid_output, reasoning, p_term, d_term = logic.calculate_pid(error)
            state = logic.update_state(horizontal_line_detected=False, trigger_zone_reached=False)
            logic.log_data(frame_count, error, p_term, d_term, pid_output, reasoning)
            
            # --- PRINT DETAILED LOGIC TO SSH CONSOLE ---
            if frame_count % 10 == 0: # Print every ~0.3s to avoid spamming SSH
                 err_str = f"{error:.1f}" if error is not None else "N/A"
                 print(f"[{frame_count}] State: {state} | Err: {err_str} | P: {p_term:.2f} | D: {d_term:.2f} | PID: {pid_output:.2f}")
                 print(f"       => {reasoning}")
            
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
                    if frame_count % 10 == 0:
                        print(f"[{frame_count}] Line Lost! Stopping...")
                    motors.stop()
                    
            elif state == "TURNING_90":
                # Spin in place — state machine in robot_logic auto-exits
                # after turn_duration frames (default 30 ≈ 1 second)
                if frame_count % 10 == 0:
                     print(f"[{frame_count}] Executing 90 deg turn ({logic.turn_counter}/{logic.turn_duration})...")
                motors.move(0.5, 0.5) # Spin in place
            
            # Hardware loop delay matches camera framerate (~30fps)
            time.sleep(0.03)
            frame_count += 1
            
    except KeyboardInterrupt:
        print("\nStopping Robot...")
    finally:
        motors.stop()
        vision.stop()
        print("Hardware Cleaned Up.")

if __name__ == "__main__":
    main()
