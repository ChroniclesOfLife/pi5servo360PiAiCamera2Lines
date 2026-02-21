from gpiozero import ContinuousRotationServo
from gpiozero.pins.lgpio import LPiFactory
import time

class RobotMotors:
    def __init__(self, left_pin=12, right_pin=13):
        """
        Initialize the robot motors using MG996R 360 servos.
        Default pins 12 and 13 (Physical 32 and 33) support hardware PWM on Pi 5.
        
        MG996R 360 Control Logic:
        - 1.0: Full speed clockwise
        - -1.0: Full speed counter-clockwise
        - 0.0: Stop
        Note: You might need to adjust 'min_pulse_width' and 'max_pulse_width' 
        specficially for MG996R if it doesn't stop exactly at 0.
        """
        self.factory = LPiFactory()
        
        # MG996R typical pulse widths: 1ms to 2ms, with 1.5ms as neutral.
        # gpiozero defaults are usually close (0.001 to 0.002).
        self.left_servo = ContinuousRotationServo(left_pin, pin_factory=self.factory)
        self.right_servo = ContinuousRotationServo(right_pin, pin_factory=self.factory)
        
        self.stop()

    def move(self, left_speed, right_speed):
        """
        Speed values are from -1.0 to 1.0.
        Note: Depending on how your motors are mounted, one might need to be inverted.
        """
        # Constrain speeds
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        # Apply speeds
        # Usually, for a tank-drive robot, one motor is flipped 180 deg,
        # so one speed might need to be negative for "forward".
        # We'll assume raw control for now.
        self.left_servo.value = left_speed
        self.right_servo.value = right_speed

    def forward(self, speed=0.5):
        # Adjust signs as per your mounting (e.g., right_speed = -speed)
        self.move(speed, -speed)

    def stop(self):
        self.left_servo.value = 0
        self.right_servo.value = 0

if __name__ == "__main__":
    # Quick test script
    print("Testing Motors...")
    motors = RobotMotors()
    
    print("Forward")
    motors.forward(0.3)
    time.sleep(2)
    
    print("Stop")
    motors.stop()
    time.sleep(1)
    
    print("Backward")
    motors.move(-0.3, 0.3)
    time.sleep(2)
    
    motors.stop()
    print("Done")
