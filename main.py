from motor_control import RobotMotors
from vision import LineFollowerVision
from robot_logic import RobotLogic
import time

def main():
    # Initialize hardware
    motors = RobotMotors(left_pin=12, right_pin=13)
    vision = LineFollowerVision(resolution=(640, 480))
    
    # Shared PID logic
    logic = RobotLogic(kp=0.005, kd=0.001, log_file="log_main.txt")

    # Control Parameters
    BASE_SPEED = 0.3 # 0.0 to 1.0 (30% speed)
    frame_count = 0
    
    print("Robot Starting in 3 seconds...")
    time.sleep(3)
    
    try:
        while True:
            # 1. Look for the line
            error = vision.get_line_error()
            pid_output, reasoning, p_term, d_term = logic.calculate_pid(error)
            state = logic.update_state(horizontal_line_detected=False, trigger_zone_reached=False)
            logic.log_data(frame_count, error, p_term, d_term, pid_output, reasoning)
            
            if state == "FOLLOWING" and error is not None:
                # 2. Apply PID steering command with safety clamp
                turn_adjust = max(-BASE_SPEED, min(BASE_SPEED, pid_output))
                
                # 3. Calculate motor speeds
                # Typical tank drive logic:
                # Turn right: left motor faster, right motor slower (or reverse)
                # Note: MG996R 360 values depend on mounting orientation.
                # In RobotMotors.forward() we used (speed, -speed).
                
                left_speed = BASE_SPEED + turn_adjust
                right_speed = -(BASE_SPEED - turn_adjust)
                
                motors.move(left_speed, right_speed)
                print(
                    f"Frame: {frame_count:5d} | Error: {error:6.1f} | "
                    f"P: {p_term:7.3f} | D: {d_term:7.3f} | PID: {pid_output:7.3f} | "
                    f"L: {left_speed:5.2f} R: {right_speed:5.2f}"
                )
            else:
                # Lost the line? Stop or spin to find it.
                print(f"Frame: {frame_count:5d} | Line Lost or state={state}. Stopping.")
                motors.stop()
            
            # Small delay for loop stability
            time.sleep(0.05)
            frame_count += 1
            
    except KeyboardInterrupt:
        print("\nStopping Robot...")
    finally:
        motors.stop()
        vision.stop()
        print("Hardware Cleaned Up.")

if __name__ == "__main__":
    main()
