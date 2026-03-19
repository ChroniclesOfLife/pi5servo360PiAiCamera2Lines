import cv2
import numpy as np
from picamera2 import Picamera2
import time

class LineFollowerVision:
    def __init__(self, resolution=(640, 480), debug_mode=False, show_stages=False, detection_mode="hybrid"):
        """
        Initialize line follower vision system.
        
        Args:
            resolution: Camera resolution (width, height)
            debug_mode: Print debug info to console
            show_stages: Save intermediate processing stages for analysis
            detection_mode: "hybrid" (default), "hough", or "contour"
        """
        self.resolution = resolution
        self.debug_mode = debug_mode
        self.show_stages = show_stages
        self.detection_mode = detection_mode.lower()
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
            print(f"[Vision] Detection Mode: {self.detection_mode}")

    def _estimate_center_error_from_hough(self, canny_edges, img_center):
        """Estimate lane-center error from Hough line segments in ROI coordinates."""
        lines = cv2.HoughLinesP(
            canny_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=40,
            minLineLength=35,
            maxLineGap=20,
        )

        hough_lines = []
        left_x = []
        right_x = []

        if lines is None:
            return None, hough_lines, None, None

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
            return None, hough_lines, None, None

        left_mean = float(np.mean(left_x))
        right_mean = float(np.mean(right_x))
        lane_center = 0.5 * (left_mean + right_mean)
        error = lane_center - img_center
        return error, hough_lines, left_mean, right_mean

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
        
        # ===== STAGE 4: Thresholding (for dark lines) =====
        _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
        
        # ===== STAGE 5: Morphological operations =====
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        thresh_clean = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh_clean = cv2.morphologyEx(thresh_clean, cv2.MORPH_OPEN, kernel)

        img_center = self.resolution[0] / 2
        
        # ===== STAGE 6: Hough Transform on Canny edges =====
        hough_error, hough_lines, left_line_x, right_line_x = self._estimate_center_error_from_hough(
            canny,
            img_center,
        )

        # ===== STAGE 7: Find contours =====
        contours, _ = cv2.findContours(thresh_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        contour_error = None
        error = None
        selected_method = "NONE"
        
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
                contour_error = midpoint - img_center
                
                if self.debug_mode and self.frame_count % 30 == 0:
                    print(f"[Frame {self.frame_count}] Contour centers: {cx1}, {cx2} | midpoint: {midpoint:.1f} | contour_error: {contour_error:.2f}")
        
        elif len(contours) == 1:
            # SAFETY: Only 1 border line is visible
            if self.debug_mode and self.frame_count % 30 == 0:
                print(f"[Frame {self.frame_count}] WARNING: Only 1 contour detected (ambiguous)")
                
        if self.detection_mode == "hough":
            error = hough_error
            selected_method = "HOUGH" if hough_error is not None else "NONE"
        elif self.detection_mode == "contour":
            error = contour_error
            selected_method = "CONTOUR" if contour_error is not None else "NONE"
        else:
            # Hybrid mode: prefer Hough coordinate estimate, fallback to contour midpoint.
            if hough_error is not None:
                error = hough_error
                selected_method = "HOUGH"
            elif contour_error is not None:
                error = contour_error
                selected_method = "CONTOUR"

        if error is None and self.debug_mode and self.frame_count % 30 == 0:
            print(f"[Frame {self.frame_count}] ERROR: Lines Lost!")

        if self.debug_mode and self.frame_count % 30 == 0:
            hough_msg = f"{hough_error:.2f}" if hough_error is not None else "N/A"
            contour_msg = f"{contour_error:.2f}" if contour_error is not None else "N/A"
            selected_msg = f"{error:.2f}" if error is not None else "N/A"
            print(
                f"[Frame {self.frame_count}] HoughErr={hough_msg} | ContourErr={contour_msg} | "
                f"Selected={selected_msg} ({selected_method})"
            )
        
        self.frame_count += 1
        
        # Return intermediate data if requested
        if return_intermediate:
            debug_dict = {
                'gray': gray,
                'blur': blur,
                'canny': canny,
                'thresh': thresh,
                'thresh_clean': thresh_clean,
                'contours': contours,
                'roi': roi,
                'contour_count': len(contours),
                'hough_lines': hough_lines,
                'hough_line_count': len(hough_lines),
                'hough_error': hough_error,
                'contour_error': contour_error,
                'selected_error': error,
                'selected_method': selected_method,
                'left_line_x': left_line_x,
                'right_line_x': right_line_x,
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

