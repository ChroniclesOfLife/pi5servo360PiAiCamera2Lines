import cv2
import numpy as np
from robot_logic import RobotLogic

def process_video(video_path="test_run.mp4"):
    """
    Offline video processing using the shared RobotLogic.
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
        roi = frame[roi_top:roi_bottom, 0:width]
        
        # Convert to HSV to find the black line
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Define range for black color (tune these based on your lighting)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        
        mask = cv2.inRange(hsv, lower_black, upper_black)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        img_center = width / 2
        error = None
        
        if len(contours) > 0:
            # Sort by area, find largest line blob
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calculate Error (Pixel distance from center)
                error = cx - img_center
                
                # Draw center of detected line
                cv2.circle(roi, (cx, cy), 10, (0, 255, 0), -1)
                cv2.line(roi, (int(img_center), cy), (cx, cy), (0, 0, 255), 2)
        
        # --- CORE BACKEND LOGIC ---
        pid_output, reasoning, p_term, d_term = logic.calculate_pid(error)
        state = logic.update_state(horizontal_line_detected=False, trigger_zone_reached=False)
        logic.log_data(frame_count, error, p_term, d_term, pid_output, reasoning)
        
        # --- OVERLAY RENDERING ---
        # Draw Dashboard on Frame
        overlay = frame.copy()
        
        # Make the dashboard background taller to fit reasoning
        dashboard_height = 250
        cv2.rectangle(overlay, (10, 10), (500, dashboard_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame) # Transparency
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(frame, f"Frame: {frame_count} | State: {state}", (20, 40), font, 0.6, (255, 255, 255), 2)
        
        if error is not None:
             cv2.putText(frame, f"Error: {error:.1f} | P: {p_term:.2f} | D: {d_term:.2f}", (20, 70), font, 0.6, (0, 255, 255), 2)
        else:
             cv2.putText(frame, f"Error: Line Lost", (20, 70), font, 0.6, (0, 0, 255), 2)
             
        cv2.putText(frame, f"Total PID Output: {pid_output:.2f}", (20, 100), font, 0.6, (255, 255, 0), 2)
        
        # Draw Reasoning Multi-line
        reasoning_lines = reasoning.split(" | ") if reasoning else ["No Data"]
        y_offset = 140
        for line in reasoning_lines:
            color = (200, 200, 200) # Light Gray
            if line.startswith("->"):
                color = (0, 255, 255) # Yellow for decisions
            cv2.putText(frame, line, (20, y_offset), font, 0.5, color, 1)
            y_offset += 25
            
        # Show ROI box on full frame
        cv2.rectangle(frame, (0, roi_top), (width, roi_bottom), (255, 0, 0), 2)
        
        cv2.imshow("Video Analyzer - Digital Twin", frame)
        cv2.imshow("Mask Filter", mask)
        
        # Delay for playback speed (e.g. 30ms for ~30fps). Press 'q' to quit.
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
            
        frame_count += 1

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    process_video()
