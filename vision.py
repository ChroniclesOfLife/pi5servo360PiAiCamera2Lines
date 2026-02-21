import cv2
import numpy as np
from picamera2 import Picamera2
import time

class LineFollowerVision:
    def __init__(self, resolution=(640, 480)):
        self.resolution = resolution
        self.picam2 = Picamera2()
        
        # Configure the camera
        config = self.picam2.create_preview_configuration(main={"format": "RGB888", "size": resolution})
        self.picam2.configure(config)
        self.picam2.start()
        
        # Region of Interest (ROI) - Bottom part of the image
        self.roi_top = int(resolution[1] * 0.6)
        self.roi_bottom = int(resolution[1] * 0.9)

    def get_line_error(self):
        """
        Captures a frame and returns the deviation from the center.
        0 means centered, negative means too far left, positive means too far right.
        """
        # Capture frame as numpy array
        frame = self.picam2.capture_array()
        
        # Crop to ROI
        roi = frame[self.roi_top:self.roi_bottom, 0:self.resolution[0]]
        
        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        
        # Gaussian blur to reduce noise
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Thresholding (adjust numbers based on line color vs floor)
        # Assuming dark lines on light background:
        _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        img_center = self.resolution[0] / 2
        
        if len(contours) >= 2:
            # Sort contours by area to find the two most prominent lines
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            
            c1 = contours[0]
            c2 = contours[1]
            
            # Get centers of both lines
            M1 = cv2.moments(c1)
            M2 = cv2.moments(c2)
            
            if M1["m00"] != 0 and M2["m00"] != 0:
                cx1 = int(M1["m10"] / M1["m00"])
                cx2 = int(M2["m10"] / M2["m00"])
                
                # Midpoint between the two lines
                midpoint = (cx1 + cx2) / 2
                error = midpoint - img_center
                return error
        
        elif len(contours) == 1:
            # Fallback if only one line is seen
            M = cv2.moments(contours[0])
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                # If we only see one line, we might want to "search" or stay relative to it
                # For now, let's treat it as the center
                return cx - img_center
                
        return None # No lines detected

    def stop(self):
        self.picam2.stop()

if __name__ == "__main__":
    vision = LineFollowerVision()
    try:
        while True:
            err = vision.get_line_error()
            print(f"Error: {err}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        vision.stop()
