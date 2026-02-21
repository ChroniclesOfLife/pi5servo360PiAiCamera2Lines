from motor_control import RobotMotors
from vision import LineFollowerVision
import time

def main():
    # Initialize hardware
    motors = RobotMotors(left_pin=12, right_pin=13)
    vision = LineFollowerVision(resolution=(640, 480))
    
    # Control Parameters
    # KP: Proportional gain. How aggressively the robot turns.
    # Base_speed: How fast it moves forward normally.
    KP = 0.005 # Adjust this: Error is in pixels (-320 to 320)
    BASE_SPEED = 0.3 # 0.0 to 1.0 (30% speed)
    
    print("Robot Starting in 3 seconds...")
    time.sleep(3)
    
    try:
        while True:
            # 1. Look for the line
            error = vision.get_line_error()
            
            if error is not None:
                # 2. Calculate steering
                # error > 0 means midpoint is to the right, need to turn right
                # error < 0 means midpoint is to the left, need to turn left
                turn_adjust = error * KP
                
                # 3. Calculate motor speeds
                # Typical tank drive logic:
                # Turn right: left motor faster, right motor slower (or reverse)
                # Note: MG996R 360 values depend on mounting orientation.
                # In RobotMotors.forward() we used (speed, -speed).
                
                left_speed = BASE_SPEED + turn_adjust
                right_speed = -(BASE_SPEED - turn_adjust)
                
                motors.move(left_speed, right_speed)
                print(f"Error: {error:4.1f} | Turn: {turn_adjust:4.2f} | L: {left_speed:4.2f} R: {right_speed:4.2f}")
            else:
                # Lost the line? Stop or spin to find it.
                print("Line Lost! Stopping...")
                motors.stop()
            
            # Small delay for loop stability
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\nStopping Robot...")
    finally:
        motors.stop()
        vision.stop()
        print("Hardware Cleaned Up.")

if __name__ == "__main__":
    main()
