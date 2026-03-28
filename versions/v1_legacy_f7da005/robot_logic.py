import time

class RobotLogic:
    def __init__(self, kp=0.5, kd=0.2, log_file="log_robot.txt", turn_duration=30):
        """
        Shared backend logic for line following (PD Controller + State Machine).
        turn_duration: number of frames to execute TURNING_90 before returning to FOLLOWING (~30 = 1s at 30fps)
        """
        self.kp = kp
        self.kd = kd
        self.prev_error = 0
        self.state = "FOLLOWING"  # States: FOLLOWING, TURNING_90
        self.log_file = log_file
        self.turn_duration = turn_duration
        self.turn_counter = 0  # counts frames spent in TURNING_90
        
        # Clear/Create log file with complete headers
        with open(self.log_file, "w", encoding="utf-8") as f:
            f.write("Frame,Time,State,Error,P_Term,D_Term,Total_Output,Reasoning\n")

    def calculate_pid(self, error):
        """
        Calculates PD controller output and a transparent reasoning string.
        Returns: (output_value, reasoning_string, p_term, d_term)
        """
        if error is None:
             return 0, "Line Lost -> Defaulting to Stop/Search", 0, 0
             
        # Derivative: Rate of change of error
        derivative = error - self.prev_error
        
        # PD Terms
        p_term = self.kp * error
        d_term = self.kd * derivative
        output = p_term + d_term
        
        # Save error for next frame
        self.prev_error = error
        
        # --- Generate Human-Readable Reasoning ---
        reasoning = []
        
        # 1. Analyze Proportional (Position) Error
        # Convention: Error > 0 means Line is on the RIGHT
        if error > 20: 
            reasoning.append("Cảm biến thấy line ở PHẢI xe (Error dương).")
        elif error < -20:
            reasoning.append("Cảm biến thấy line ở TRÁI xe (Error âm).")
        else:
            reasoning.append("Xe đang ĐI THẢNG giữa line.")
            
        # 2. Analyze Derivative (Momentum/Trend) Error
        if abs(derivative) > 10:
            if derivative * error > 0:
                 reasoning.append(f"Xe đang LỆCH NHANH HƠN (D={derivative:.1f}), cần phanh gấp.")
            else:
                 reasoning.append(f"Xe đang TRỞ LẠI TÂM (D={derivative:.1f}), giảm góc lái để chống lắc.")
                 
        # 3. Final Action Decision
        # Convention: Output > 0 means TURN RIGHT
        if output > 0.5:
            reasoning.append("-> QUYẾT ĐỊNH: Rẽ Phải Mạnh.")
        elif output < -0.5:
            reasoning.append("-> QUYẾT ĐỊNH: Rẽ Trái Mạnh.")
        elif output > 0.1:
            reasoning.append("-> QUYẾT ĐỊNH: Rẽ Phải Nhẹ.")
        elif output < -0.1:
            reasoning.append("-> QUYẾT ĐỊNH: Rẽ Trái Nhẹ.")
        else:
            reasoning.append("-> QUYẾT ĐỊNH: Giữ Thẳng Lái.")

        final_reasoning = " | ".join(reasoning)
        
        return output, final_reasoning, p_term, d_term

    def update_state(self, horizontal_line_detected, trigger_zone_reached):
        """
        State Machine: transitions between FOLLOWING and TURNING_90.
        FOLLOWING → TURNING_90: when a horizontal line is seen inside the trigger zone.
        TURNING_90 → FOLLOWING: after turn_duration frames have elapsed.
        """
        if self.state == "FOLLOWING" and horizontal_line_detected and trigger_zone_reached:
            self.state = "TURNING_90"
            self.turn_counter = 0
            self.prev_error = 0  # reset derivative to prevent spike on re-entry
        
        elif self.state == "TURNING_90":
            self.turn_counter += 1
            if self.turn_counter >= self.turn_duration:
                self.state = "FOLLOWING"
                self.prev_error = 0  # clean start for PD controller
            
        return self.state

    def log_data(self, frame_count, error, p_term, d_term, output, reasoning):
        """
        Logs detailed parameters for transparent monitoring and debugging.
        """
        current_time = time.time()
        err_val = error if error is not None else 0
        
        # Format: Frame,Time,State,Error,P,D,Output,Reasoning
        # Quote reasoning to prevent CSV corruption (it contains commas and pipe chars)
        safe_reasoning = reasoning.replace('"', '""')  # escape any internal quotes
        log_entry = f'{frame_count},{current_time:.2f},{self.state},{err_val:.2f},{p_term:.2f},{d_term:.2f},{output:.2f},"{safe_reasoning}"\n'
        
        with open(self.log_file, "a", encoding="utf-8") as f:
            f.write(log_entry)

