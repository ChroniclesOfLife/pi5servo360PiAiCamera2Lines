# Debug & Visualization Guide

## Overview

This guide covers the enhanced debugging and visualization capabilities added to the pi5servo360PiAiCamera2Lines project. The system now provides multi-stage algorithm visualization, slow-motion debugging, and detailed algorithm stage output at every step.

## Key Features

### 1. **Video Processor** - Offline Video Analysis with Full Pipeline Visualization

The video processor is the best tool for understanding the complete algorithm pipeline. It shows every processing stage side-by-side.

#### Running the Processor

```bash
python video_processor.py
```

#### What You'll See

**Main Windows:**
1. **"1_VIDEO_ANALYZER - Main Frame"** - Full frame with PID logic overlay
   - Dashboard showing Frame, State, Error, PID values
   - ROI highlighted with cyan rectangle
   - Red detection contours when lines are found
   - Green circle + green center point for detected midpoint
   - Reasoning string showing algorithm decisions

2. **"2_ROI_WITH_DETECTIONS"** - Region of Interest zoom
   - Clean view of the detection area
   - Red contours showing detected black lines
   - Green circle at the computed center
   - Red error line from image center to detection

3. **"3_PIPELINE_STAGES"** (if enabled) - Five-panel algorithm pipeline
   - **Panel 1: Grayscale** - RGB to grayscale conversion
   - **Panel 2: Canny Edges** - Edge detection output (50-150 threshold)
   - **Panel 3: HSV Mask** - Black color detection in HSV space
   - **Panel 4: Clean Mask** - After morphological cleaning (open/close)
  - **Panel 5: Hough Segments** - Line segments used for coordinate extraction

#### Parameters in Code

```python
process_video(
    video_path="test_run.mp4",           # Input video file
    debug_mode=True,                      # Print console debug info
    slow_motion_factor=2,                 # 1=realtime, 2=half-speed, 3=third-speed
    show_pipeline=True                    # Show algorithm stages
)
```

#### Keyboard Controls

- **Q** - Quit playback
- **P** - Pause/unpause (Press SPACE to resume)

#### Algorithm Stages Explained

```
Raw Frame
    ↓
[STAGE 1] RGB → Grayscale
    ↓
[STAGE 2] Gaussian Blur (5×5 kernel)
    ↓
[STAGE 3] Canny Edge Detection (50-150 thresholds)
    ↓
[STAGE 4] HSV Color Masking (black detection)
    ↓
[STAGE 5] Morphological Operations (CLOSE + OPEN)
    ↓
[STAGE 6] Hough Transform (line segments + left/right coordinate estimate)
    ↓
[STAGE 7] Contour Finding (mask-based fallback)
    ↓
[STAGE 8] Select Error (Hybrid: Hough first, contour fallback)
  ↓
[STAGE 9] PD Controller (P_term + D_term = Steering)
    ↓
Output: Motor Command
```

---

### 2. **Simulator** - Real-Time Virtual Robot Debugging

The digital twin simulator shows the robot physics in real-time with adjustable slow-motion.

#### Running the Simulator

```bash
python simulator.py
```

#### Default Configuration

The simulator runs with **2x slow-motion** by default for better visualization. Edit [line 148](simulator.py#L148) to change:

```python
main(slow_motion_factor=2, debug_verbose=True)
```

**Slow-motion options:**
- `slow_motion_factor=1` → 60 FPS (real-time)
- `slow_motion_factor=2` → 30 FPS (2x slow)
- `slow_motion_factor=3` → 20 FPS (3x slow)
- `slow_motion_factor=4` → 15 FPS (4x slow)

#### What You'll See

**Left Side: Track Map (800×600)**
- **Black nested rectangles** - Race track (simulated)
- **Red box** - Robot car
- **Yellow arrow** - Robot heading direction
- **Cyan line** - Virtual camera scanline
- **Green dot** - Detected line position
- **Red area** - Detection region (where line is found)

**Right Side: Debug Dashboard**
- **Simulator Debug** - Frame count, FPS, state, detection status
- **PID Algorithm** - Error, P_term, D_term, Total PID, turn adjustment
- **Reasoning** - Human-readable algorithm decisions
- **Legend** - Color coding explanation

#### Keyboard Controls

- **ESC** - Quit simulator
- **P** - Pause simulation

#### Debug Dashboard Explained

```
Frame       → Current frame number
FPS         → Running frame rate with slow-motion factor
State       → Current FSM state (FOLLOWING, TURNING_90, etc.)
Detection   → ✓ YES (line found) or ✗ LOST (no line)
Car Angle   → Current heading angle in degrees
Car Pos     → X, Y position on the track

Error       → Pixel deviation from center (negative=left, positive=right)
P_term      → Proportional response = Kp × Error
D_term      → Derivative response = Kd × dError/dt
Total PID   → P_term + D_term (motor command)
Turn Adjust → Constrained PID output (-3.0 to 3.0)

Reasoning   → Vietnamese explanations of algorithm logic
```

---

### 3. **Vision Module (Pi)** - Camera Pipeline with Debug Output

Enhanced Pi camera module with algorithm stage visibility and debug logging.

#### Initialization with Debug

```python
from vision import LineFollowerVision

vision = LineFollowerVision(
    resolution=(640, 480),
    debug_mode=True,           # Print frame-by-frame status
    show_stages=True           # Save intermediate stages
)
```

#### Debug Output Example

```
[Vision] Initialized with resolution (640, 480)
[Vision] ROI: rows 288 to 432
[Vision] Debug Mode: ON | Show Stages: True

[Frame 30] Two-line detection: centers at 285, 315 | midpoint: 300.0 | error: -20.0
[Frame 60] Two-line detection: centers at 290, 310 | midpoint: 300.0 | error: 0.0
[Frame 90] WARNING: Only 1 contour detected (ambiguous)
[Frame 120] ERROR: Lines Lost!
```

#### Getting Intermediate Stages

For advanced debugging, get all processing stages:

```python
error, debug_dict = vision.get_line_error(return_intermediate=True)

# Access intermediate stages:
gray = debug_dict['gray']              # Grayscale image
blur = debug_dict['blur']              # Blurred image
canny = debug_dict['canny']            # Canny edges
thresh = debug_dict['thresh']          # Threshold mask
thresh_clean = debug_dict['thresh_clean']  # Cleaned mask
contours = debug_dict['contours']      # Found contours
contour_count = debug_dict['contour_count']  # Number of blobs
hough_lines = debug_dict['hough_lines']      # Hough line segments
hough_error = debug_dict['hough_error']      # Hough-derived center error
selected_error = debug_dict['selected_error']  # Error used by controller
selected_method = debug_dict['selected_method']  # HOUGH or CONTOUR
```

---

### 4. **Main Pi (main_pi.py)** - Real Robot with Enhanced Logging

The main robot execution now includes comprehensive debug logging and statistics.

#### Running on Raspberry Pi

```bash
python main_pi.py
```

#### Debug Output Example

```
============================================================
ROBOT LINE FOLLOWER - Raspberry Pi 5 with IMX500 AI Camera
============================================================
[Config] BASE_SPEED: 0.3
[Config] Kp: 0.005, Kd: 0.001
[Config] Left Servo: GPIO 12, Right Servo: GPIO 13
[Status] Hardware initialized and ready

Robot Starting in 3 seconds... Digital Twin Logic Engaged.

[    0] State: FOLLOWING   | Error:   0.0 | P:   0.000 | D:   0.000 | PID:   0.000
        => Xe đang ĐI THẢNG giữa line. | -> QUYẾT ĐỊNH: Giữ Thẳng Lái.
[   10] State: FOLLOWING   | Error: -15.3 | P:  -0.077 | D:  -0.005 | PID:  -0.082
        => Cảm biến thấy line ở TRÁI xe (Error âm). | -> QUYẾT ĐỊNH: Rẽ Trái Nhẹ.
[   20] State: FOLLOWING   | Error:   5.2 | P:   0.026 | D:   0.010 | PID:   0.036
        => Cảm biến thấy line ở PHẢI xe (Error dương). | -> QUYẾT ĐỊNH: Rẽ Phải Nhẹ.

============================================================
EXECUTION SUMMARY
============================================================
Total Frames: 2150
Line Detection Loss Events: 12 (0.6%)
Log File: log_pibot.txt
Hardware Cleaned Up.
============================================================
```

#### Parameters

```python
main(
    debug_mode=True,        # Enable detailed logging
    verbose_period=10       # Print status every N frames (10 = ~0.3s at 30fps)
)
```

---

## Troubleshooting Guide

### Issue: "Line Lost" Frequently

**Diagnosis Steps:**
1. Run video processor with `show_pipeline=True`
2. Check "3_PIPELINE_STAGES" window
3. Look at **HSV Mask** panel - is the black line visible?
4. If black line is there but not in "Clean Mask" panel → morphological operation is too aggressive

**Solution Options:**
- Adjust kernel size in video_processor.py line 81:
  ```python
  kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))  # Try smaller
  ```
- Adjust threshold in line 51:
  ```python
  lower_black = np.array([0, 0, 0])
  upper_black = np.array([180, 255, 100])  # Increase upper V value
  ```

### Issue: Robot Oscillates/Jerks

**Diagnosis:**
1. Check "PID Algorithm" section in simulator
2. Is D_term very large? → Derivative is too aggressive
3. Is Error changing wildly? → Noisy sensor data

**Solution:**
- Reduce Kd value in robot_logic.py:
  ```python
  logic = RobotLogic(kp=0.005, kd=0.0005)  # Lower from 0.001
  ```

### Issue: Robot Misses Turns or Won't Follow

**Diagnosis:**
1. Run simulator and watch the green dot (detected center)
2. Check if Error is calculated correctly
3. Is PID output sufficient to turn?

**Solution:**
- Increase Kp in robot_logic.py:
  ```python
  logic = RobotLogic(kp=0.01, kd=0.001)  # Higher from 0.005
  ```

### Issue: Can't See Pipeline Stages

**Cause:** `show_pipeline=False` in video_processor.py line 194

**Fix:**
```python
process_video(debug_mode=True, slow_motion_factor=2, show_pipeline=True)
```

---

## Understanding the Algorithm

### PD Controller Logic

The system uses a **Proportional-Derivative (PD)** controller:

```
Error = Detected_Center - Image_Center
  (negative = line to left, positive = line to right)

P_Term = Kp × Error
  (larger error → stronger turn)

D_Term = Kd × (Error - Previous_Error)
  (derivative = rate of change, prevents oscillation)

Output = P_Term + D_Term
  (positive = turn right, negative = turn left)
```

### State Machine

```
START
  ↓
FOLLOWING
  - Detect two black lines
  - Calculate midpoint error
  - Apply PD controller
  - Stay centered between lines
  ↓ [on horizontal line AND in trigger zone]
TURNING_90
  - Spin in place (0.5, 0.5 speed)
  - Count 30 frames (~1 second)
  ↓ [turn_counter >= turn_duration]
FOLLOWING
  - Resume line following
```

---

## Performance Tuning Tips

### For Faster Response
- Increase Kp (e.g., 0.005 → 0.01)
- Increase Kd (e.g., 0.001 → 0.002)
- **Caution:** Too high causes oscillation

### For Smoother Motion
- Decrease Kd (e.g., 0.001 → 0.0005)
- Increase blur kernel (e.g., 5 → 7)
- Reduce slow_motion_factor in simulator

### For Better Line Detection
- Test different HSV ranges in video processor
- Adjust Canny thresholds (50, 150)
- Check lighting conditions in ROI

---

## Log File Analysis

Each mode generates a log file:
- **video_processor.py** → `log_video.txt`
- **main_pi.py** → `log_pibot.txt`
- **simulator.py** → `log_robot.txt`

### Log Format

```csv
Frame,Time,State,Error,P_Term,D_Term,Total_Output,Reasoning
0,1234567890.12,FOLLOWING,-15.30,-0.077,-0.001,-0.078,"Cảm biến thấy line ở TRÁI xe..."
1,1234567890.15,FOLLOWING,-14.50,-0.073,0.003,-0.070,"Xe đang LỆCH NHANH HƠN..."
2,1234567890.18,FOLLOWING,-13.20,-0.066,0.011,-0.055,"Xe đang TRỞ LẠI TÂM..."
```

**Analysis tips:**
- Plot Error over time to see line following smoothness
- Check P_Term vs D_Term ratio to understand control balance
- Look at Reasoning changes to understand decision transitions

---

## Demo Workflow

For a complete demonstration of all features:

1. **Start with Video Processor:**
   ```bash
   python video_processor.py
   ```
   - Watch camera input pipeline
   - Understand algorithm stages
   - See PID output in action

2. **Run Simulator:**
   ```bash
   python simulator.py
   ```
   - See robot physics
   - Watch line following behavior
   - Observe PID corrections

3. **Deploy on Pi:**
   ```bash
   python main_pi.py
   ```
   - Real robot with video input
   - Same logic as simulator
   - Hardware integration test

---

## File Changes Summary

### Enhanced Files

| File | Changes |
|------|---------|
| `video_processor.py` | Added Canny edges, morphological ops, Hough transform + coordinate extraction, pipeline stages display, slow-motion support |
| `simulator.py` | Added slow-motion factor, red detection area display, enhanced dashboard, pause control |
| `vision.py` | Added debug_mode, show_stages, Hough/contour/hybrid detection modes, intermediate data return, frame counting |
| `main_pi.py` | Added debug output, statistics tracking, enhanced console logging, explicit hybrid detection mode |

### New Concepts

- **Slow-motion factor** - Divide FPS by this number
- **Pipeline visualization** - See algorithm stages side-by-side
- **Detection regions** - Red areas showing where lines are found
- **Debug dictionary** - Access intermediate processing stages

---

## Next Steps for Debugging

1. **Identify the issue** - Use video processor pipeline view
2. **Isolate the stage** - Test in simulator first
3. **Adjust parameters** - Modify Kp, Kd, or image processing
4. **Log analysis** - Check output log files for patterns
5. **Test on robot** - Deploy to Pi when simulator works well

---

*Last Updated: 2026-03-18*
*Project: pi5servo360PiAiCamera2Lines*
*Architecture: Digital Twin with unified backend logic*
