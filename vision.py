import cv2
import numpy as np
from picamera2 import Picamera2
import time

class LineFollowerVision:
    def __init__(self, resolution=(640, 480), debug_mode=False, show_stages=False):
        """
        Initialize line follower vision system using only:
        Gaussian Blur + Canny + Hough line detection.
        
        Args:
            resolution: Camera resolution (width, height)
            debug_mode: Print debug info to console
            show_stages: Save intermediate processing stages for analysis
        """
        self.resolution = resolution
        self.debug_mode = debug_mode
        self.show_stages = show_stages
        self.picam2 = Picamera2()
        
        # Configure the camera
        config = self.picam2.create_preview_configuration(main={"format": "RGB888", "size": resolution})
        self.picam2.configure(config)
        self.picam2.start()
        
        # Region of Interest (ROI) - Bottom part of the image
        self.roi_top = int(resolution[1] * 0.6)
        self.roi_bottom = int(resolution[1] * 0.9)
        
        # Frame counter for logging
        self.frame_count = 0
        
        if debug_mode:
            print(f"[Vision] Initialized with resolution {resolution}")
            print(f"[Vision] ROI: rows {self.roi_top} to {self.roi_bottom}")
            print(f"[Vision] Debug Mode: ON | Show Stages: {show_stages}")
            print("[Vision] Detection Mode: HOUGH_ONLY")

    def _estimate_center_error_from_hough(self, canny_edges, img_center):
        """Estimate lane-center error from Hough line segments in ROI coordinates."""
        lines = cv2.HoughLinesP(
            canny_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=38,
            minLineLength=30,
            maxLineGap=18,
        )

        hough_lines = []
        left_x = []
        right_x = []

        if lines is None:
            return None, hough_lines, None, None, None

        for line in lines[:, 0]:
            x1, y1, x2, y2 = map(int, line)
            dx = x2 - x1
            dy = y2 - y1

            # Keep near-vertical line segments.
            angle_deg = abs(np.degrees(np.arctan2(dy, dx)))
            verticality = min(angle_deg, 180 - angle_deg)
            if verticality < 55:
                continue

            x_mid = 0.5 * (x1 + x2)
            hough_lines.append((x1, y1, x2, y2))

            if x_mid < img_center:
                left_x.append(x_mid)
            else:
                right_x.append(x_mid)

        if not left_x or not right_x:
            return None, hough_lines, None, None, None

        left_mean = float(np.mean(left_x))
        right_mean = float(np.mean(right_x))
        lane_center = 0.5 * (left_mean + right_mean)
        error = lane_center - img_center
        return error, hough_lines, left_mean, right_mean, lane_center

    def get_line_error(self, return_intermediate=False):
        """
        Captures a frame and returns the deviation from the center.
        0 means centered, negative means too far left, positive means too far right.
        
        Args:
            return_intermediate (bool): If True, return intermediate processing data
            
        Returns:
            error (float or None): Pixel error from center
            OR if return_intermediate=True:
            (error, debug_dict): where debug_dict contains intermediate stages
        """
        # Capture frame as numpy array
        frame = self.picam2.capture_array()
        
        # Crop to ROI
        roi = frame[self.roi_top:self.roi_bottom, 0:self.resolution[0]]
        
        # ===== STAGE 1: RGB to Grayscale =====
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        
        # ===== STAGE 2: Gaussian Blur =====
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # ===== STAGE 3: Canny Edge Detection =====
        canny = cv2.Canny(blur, 50, 150)
        
        img_center = self.resolution[0] / 2
        
        # ===== STAGE 4: Hough Transform on Canny edges =====
        error, hough_lines, left_line_x, right_line_x, lane_center = self._estimate_center_error_from_hough(
            canny,
            img_center,
        )

        if error is None and self.debug_mode and self.frame_count % 30 == 0:
            print(f"[Frame {self.frame_count}] ERROR: Lines Lost!")

        if self.debug_mode and self.frame_count % 30 == 0:
            hough_msg = f"{error:.2f}" if error is not None else "N/A"
            left_msg = f"{left_line_x:.1f}" if left_line_x is not None else "N/A"
            right_msg = f"{right_line_x:.1f}" if right_line_x is not None else "N/A"
            center_msg = f"{lane_center:.1f}" if lane_center is not None else "N/A"
            print(
                f"[Frame {self.frame_count}] HoughErr={hough_msg} | "
                f"LeftX={left_msg} | RightX={right_msg} | LaneCenter={center_msg}"
            )
        
        self.frame_count += 1
        
        # Return intermediate data if requested
        if return_intermediate:
            debug_dict = {
                'gray': gray,
                'blur': blur,
                'canny': canny,
                'roi': roi,
                'hough_lines': hough_lines,
                'hough_line_count': len(hough_lines),
                'hough_error': error,
                'left_line_x': left_line_x,
                'right_line_x': right_line_x,
                'lane_center_x': lane_center,
                'image_center_x': img_center,
            }
            return error, debug_dict
        
        return error

    def stop(self):
        """Stop the camera and clean up."""
        self.picam2.stop()
        if self.debug_mode:
            print(f"[Vision] Stopped after {self.frame_count} frames")

if __name__ == "__main__":
    vision = LineFollowerVision(debug_mode=True, show_stages=True)
    try:
        print("[Main] Running vision system. Press Ctrl+C to stop.")
        for i in range(300):
            err = vision.get_line_error()
            if i % 30 == 0:
                print(f"[{i}] Error: {err}")
            time.sleep(0.03)  # 30ms = ~33fps
    except KeyboardInterrupt:
        print("\n[Main] Stopping...")
    finally:
        vision.stop()

