import cv2
import numpy as np
from robot_logic import RobotLogic


def estimate_center_error_from_hough(canny_edges, img_center, min_line_length=35, max_line_gap=20):
    """Estimate lane-center error from Hough line segments in the ROI."""
    lines = cv2.HoughLinesP(
        canny_edges,
        rho=1,
        theta=np.pi / 180,
        threshold=40,
        minLineLength=min_line_length,
        maxLineGap=max_line_gap,
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

        # Keep near-vertical segments and reject near-horizontal clutter.
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

def process_video(video_path="test_run.mp4", debug_mode=True, slow_motion_factor=2, show_pipeline=True):
    """
    Offline video processing using the shared RobotLogic.
    
    Args:
        video_path (str): Path to input video file
        debug_mode (bool): Enable detailed debug output
        slow_motion_factor (int): Multiply frame delay (higher = slower playback)
        show_pipeline (bool): Show algorithm pipeline stages (Canny, masks, etc.)
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file {video_path}")
        return
        
    logic = RobotLogic(kp=0.5, kd=0.1, log_file="log_video.txt")
    frame_count = 0
    
    # Define ROI (Region Of Interest)
    roi_top_ratio = 0.5
    roi_bottom_ratio = 0.9
    
    # Playback delay (ms) - increased for slow motion
    base_delay = 50  # ~20fps base
    playback_delay = base_delay * slow_motion_factor
    
    print(f"[DEBUG] Starting video processor with {slow_motion_factor}x slow motion")
    print(f"[DEBUG] Pipeline visualization: {'ENABLED' if show_pipeline else 'DISABLED'}")
    print(f"[DEBUG] Playback delay: {playback_delay}ms")

    while True:
        ret, frame = cap.read()
        
        # Loop video if it ends
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = cap.read()
            if not ret: break # Break completely if it's not a video
            
        height, width = frame.shape[:2]
        
        # Crop to ROI
        roi_top = int(height * roi_top_ratio)
        roi_bottom = int(height * roi_bottom_ratio)
        roi = frame[roi_top:roi_bottom, 0:width].copy()
        
        # ==== STAGE 1: RGB to Grayscale ====
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # ==== STAGE 2: Gaussian Blur ====
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # ==== STAGE 3: Canny Edge Detection ====
        canny_edges = cv2.Canny(blur, 50, 150)
        
        # ==== STAGE 4: Convert to HSV to find the black line ====
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Define range for black color (tune these based on your lighting)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        
        mask = cv2.inRange(hsv, lower_black, upper_black)
        
        # ==== STAGE 5: Morphological operations to clean mask ====
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel)
        
        # ==== STAGE 6: Hough Transform on Canny edges ====
        img_center = width / 2
        hough_error, hough_lines, left_line_x, right_line_x = estimate_center_error_from_hough(
            canny_edges,
            img_center,
        )

        # ==== STAGE 7: Find contours ====
        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        contour_error = None
        error = None
        detection_method = "NONE"
        
        if len(contours) > 0:
            # Sort by area, find largest line blob
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calculate contour-based error (pixel distance from center)
                contour_error = cx - img_center
                
                # Draw center of detected line on ROI
                cv2.circle(roi, (cx, cy), 12, (0, 255, 0), -1)  # Green circle at detection
                cv2.circle(roi, (cx, cy), 12, (0, 200, 0), 2)   # Green outline
                
                # Draw error line (red line from center to detected point)
                cv2.line(roi, (int(img_center), cy), (cx, cy), (0, 0, 255), 3)
                
                # Draw detection radius in red semi-transparent
                roi_overlay = roi.copy()
                cv2.drawContours(roi_overlay, [c], 0, (0, 0, 255), 3)  # Red contour
                cv2.addWeighted(roi_overlay, 0.4, roi, 0.6, 0, roi)

        # Draw Hough segments and coordinate guide lines.
        for x1, y1, x2, y2 in hough_lines:
            cv2.line(roi, (x1, y1), (x2, y2), (255, 0, 255), 2)

        if left_line_x is not None and right_line_x is not None:
            lane_mid_x = int((left_line_x + right_line_x) / 2)
            cv2.line(roi, (int(left_line_x), 0), (int(left_line_x), roi.shape[0] - 1), (255, 200, 0), 1)
            cv2.line(roi, (int(right_line_x), 0), (int(right_line_x), roi.shape[0] - 1), (255, 200, 0), 1)
            cv2.line(roi, (lane_mid_x, 0), (lane_mid_x, roi.shape[0] - 1), (0, 255, 255), 2)
            cv2.circle(roi, (lane_mid_x, roi.shape[0] // 2), 8, (0, 255, 255), -1)

        # Prefer Hough-coordinate error for control, fallback to contour center.
        if hough_error is not None:
            error = hough_error
            detection_method = "HOUGH"
        elif contour_error is not None:
            error = contour_error
            detection_method = "CONTOUR"
        
        # --- CORE BACKEND LOGIC ---
        pid_output, reasoning, p_term, d_term = logic.calculate_pid(error)
        state = logic.update_state(horizontal_line_detected=False, trigger_zone_reached=False)
        logic.log_data(frame_count, error, p_term, d_term, pid_output, reasoning)
        
        # --- RENDER MAIN FRAME WITH DASHBOARD OVERLAY ---
        display_frame = frame.copy()
        overlay = display_frame.copy()
        
        # Draw semi-transparent dashboard background
        dashboard_height = 280
        cv2.rectangle(overlay, (10, 10), (560, dashboard_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, display_frame, 0.3, 0, display_frame)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # ---- Dashboard Content ----
        cv2.putText(display_frame, f"Frame: {frame_count} | State: {state}", 
                   (20, 40), font, 0.7, (255, 255, 255), 2)
        
        cv2.putText(display_frame, f"ROI: [{roi_top}-{roi_bottom}]", 
                   (20, 70), font, 0.5, (200, 200, 200), 1)
        
        if error is not None:
             cv2.putText(display_frame, f"Error: {error:.1f}px | P: {p_term:.3f} | D: {d_term:.3f}", 
                        (20, 100), font, 0.6, (0, 255, 255), 2)
        else:
             cv2.putText(display_frame, f"Error: LINE LOST", 
                        (20, 100), font, 0.6, (0, 0, 255), 2)
             
        cv2.putText(display_frame, f"PID Output (Steering): {pid_output:.3f}", 
                   (20, 130), font, 0.6, (255, 255, 0), 2)
        cv2.putText(display_frame, f"Detections: {len(contours)} blob(s) | Hough Segments: {len(hough_lines)}", 
                   (20, 160), font, 0.5, (200, 200, 200), 1)
        cv2.putText(display_frame, f"Method: {detection_method}",
               (20, 182), font, 0.5, (200, 255, 200), 1)
        
        # Draw Reasoning Multi-line
        reasoning_lines = reasoning.split(" | ") if reasoning else ["No Data"]
        y_offset = 208
        for line in reasoning_lines:
            color = (200, 200, 200) # Light Gray
            if line.startswith("->"):
                color = (0, 255, 255) # Cyan for decisions
            cv2.putText(display_frame, line, (20, y_offset), font, 0.45, color, 1)
            y_offset += 22
            
        # Highlight ROI box on full frame (cyan rectangle)
        cv2.rectangle(display_frame, (0, roi_top), (width, roi_bottom), (255, 255, 0), 3)
        
        # Show main visualization
        cv2.imshow("1_VIDEO_ANALYZER - Main Frame (PID Logic)", display_frame)
        
        # Show ROI with detections
        cv2.imshow("2_ROI_WITH_DETECTIONS - Red=Detection Area, Green=Center", roi)
        
        # --- PIPELINE VISUALIZATION (if enabled) ---
        if show_pipeline:
            # Create pipeline grid showing processing stages
            
            # Grayscale stage
            gray_3ch = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.putText(gray_3ch, "Grayscale", (5, 25), font, 0.6, (255, 255, 255), 1)
            
            # Canny stage
            canny_3ch = cv2.cvtColor(canny_edges, cv2.COLOR_GRAY2BGR)
            cv2.putText(canny_3ch, "Canny Edges (50-150)", (5, 25), font, 0.6, (255, 255, 255), 1)
            
            # HSV mask stage
            mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            cv2.putText(mask_3ch, "Black HSV Mask", (5, 25), font, 0.6, (255, 255, 255), 1)
            
            # Clean mask stage
            mask_clean_3ch = cv2.cvtColor(mask_clean, cv2.COLOR_GRAY2BGR)
            cv2.putText(mask_clean_3ch, "Morpho Cleaned", (5, 25), font, 0.6, (255, 255, 255), 1)
            
            # Hough stage preview
            hough_3ch = cv2.cvtColor(canny_edges, cv2.COLOR_GRAY2BGR)
            for x1, y1, x2, y2 in hough_lines:
                cv2.line(hough_3ch, (x1, y1), (x2, y2), (255, 0, 255), 2)
            cv2.putText(hough_3ch, "Hough Segments", (5, 25), font, 0.6, (255, 255, 255), 1)

            # Create horizontal tile of all stages
            h = roi.shape[0]
            w = roi.shape[1] // 5
            
            # Resize all pipeline stages to quarter width
            gray_resized = cv2.resize(gray_3ch, (w, h))
            canny_resized = cv2.resize(canny_3ch, (w, h))
            mask_resized = cv2.resize(mask_3ch, (w, h))
            mask_clean_resized = cv2.resize(mask_clean_3ch, (w, h))
            hough_resized = cv2.resize(hough_3ch, (w, h))
            
            # Horizontally stack
            pipeline_stages = np.hstack([gray_resized, canny_resized, mask_resized, mask_clean_resized, hough_resized])
            
            cv2.imshow("3_PIPELINE_STAGES - Gray | Canny | HSV Mask | Clean Mask | Hough", pipeline_stages)
        
        # Playback control
        key = cv2.waitKey(playback_delay) & 0xFF
        if key == ord('q'):
            print(f"[DEBUG] User quit at frame {frame_count}")
            break
        if key == ord('p'):
            print(f"[DEBUG] Paused at frame {frame_count}. Press SPACE to continue or Q to quit...")
            while True:
                k = cv2.waitKey(0) & 0xFF
                if k == ord(' '):
                    break
                if k == ord('q'):
                    key = ord('q')
                    break
            if key == ord('q'):
                break
            
        frame_count += 1
        
        if debug_mode and frame_count % 30 == 0:
            err_msg = f"{error:.1f}" if error is not None else "N/A"
            print(f"[Frame {frame_count}] Error: {err_msg} | PID: {pid_output:.3f} | State: {state} | Method: {detection_method}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"[DEBUG] Processing complete. Processed {frame_count} frames.")

if __name__ == "__main__":
    # video_processor with 2x slow motion and pipeline visualization
    process_video(debug_mode=True, slow_motion_factor=2, show_pipeline=True)
