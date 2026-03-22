# Pi5 Servo + AI Camera Line Follower

This project is a line-following robot system for Raspberry Pi 5 with MG996R 360 servos and Pi Camera (Picamera2).

It now uses one strict perception/control stack across the project:
- Gaussian Blur
- Canny Edge Detection (with hysteresis thresholds)
- Hough Transform (line segments + lane center coordinates)
- PID (PD form in code: P + D)

No contour, HSV mask, threshold segmentation, or morphology are used in the active vision pipeline.

## Why This Design
The goal is robust and explainable lane-center estimation for a top-down track view (black lane boundaries on a light floor), while keeping the algorithm simple enough to present clearly.

The pipeline is:
1. Convert ROI to grayscale
2. Apply Gaussian blur to reduce sensor noise
3. Run Canny edge detection (low/high hysteresis thresholds)
4. Run probabilistic Hough transform to extract line segments
5. Split Hough segments into left/right lane groups
6. Compute lane center from left/right x-coordinate means
7. Compute steering error: lane_center - image_center
8. PID logic computes steering correction
9. Motor mapping applies correction to left/right servos

## Project Structure
- `main_pi.py`
  - Production runtime for Raspberry Pi 5 hardware.
  - Reads error from `LineFollowerVision` and sends PID command to motors.
- `main.py`
  - Simplified hardware runtime that also uses shared PID logic.
- `vision.py`
  - Camera perception module (Gaussian + Canny + Hough only).
  - Returns lane center error in pixels.
- `video_processor.py`
  - Offline analyzer for recorded video.
  - Very slow playback by default for presentations.
  - Multi-window debug output for each processing stage.
- `robot_logic.py`
  - Shared PID/PD and state machine logic.
  - English debug reasoning and CSV logging.
- `motor_control.py`
  - MG996R servo abstraction via gpiozero/lgpio.
- `simulator.py`
  - Digital twin simulator using shared PID logic for behavior tuning.
- `requirements.txt`, `environment.yml`
  - Environment/dependency setup.

## Data Flow
Camera frame -> ROI crop -> Gray -> Gaussian -> Canny -> Hough -> lane center coordinates -> error -> PID -> motor speeds.

## Core Math
Let:
- $e_t$: lane center error in pixels at frame $t$
- $K_p, K_d$: gains

Controller:
$$
P_t = K_p \cdot e_t
$$
$$
D_t = K_d \cdot (e_t - e_{t-1})
$$
$$
u_t = P_t + D_t
$$

Where $u_t$ is the steering correction.

Motor mapping (tank style with one side inverted by mounting):
- `left_speed = BASE_SPEED + clamp(u_t)`
- `right_speed = -(BASE_SPEED - clamp(u_t))`

## Runtime Files and Purpose
### 1) Physical Robot
Run:
```bash
python main_pi.py
```
What it does:
- Initializes GPIO servos and camera
- Computes error from Hough lane center
- Runs PID each frame
- Logs detailed telemetry

### 2) Offline Video Debug (Recommended for demos)
Run:
```bash
python video_processor.py
```
What it does:
- Opens a video file (`test_run.mp4` by default)
- Uses strict Gray->Blur->Canny->Hough->PID path
- Shows:
  - Main dashboard window
  - ROI overlay window with Hough lines and lane center markers
  - Stage window (`Gray | Blur | Canny | Hough`)

Default playback is intentionally very slow for explanation to management.

Controls:
- `q`: quit
- `p`: pause
- `space`: resume after pause

### 3) Simulator
Run:
```bash
python simulator.py
```
Uses the same shared PID logic for behavior and tuning.

## Key Configuration Points
### `vision.py`
- ROI location: bottom area of frame (`roi_top`, `roi_bottom`)
- Canny thresholds: `50`, `150`
- Hough params: `threshold`, `minLineLength`, `maxLineGap`
- Vertical filtering: rejects low-angle segments

### `video_processor.py`
- Slowdown: `slow_motion_factor` (default set high for presentation)
- Base delay: `base_delay`
- Debug print cadence: every 30 frames

### `robot_logic.py`
- Gains: `kp`, `kd`
- State machine: `FOLLOWING`, `TURNING_90`
- CSV logs for post-run analysis

## Logs
CSV log files include:
- Frame index
- Timestamp
- State
- Error
- P term
- D term
- Total output
- Human-readable reasoning

## English Debug Reasoning
All decision text is now in English, for professional reporting and presentation.

Examples:
- `Lane center detected to the RIGHT of robot center (positive error).`
- `-> DECISION: Turn LEFT gently.`

## Setup
### Conda (recommended)
```bash
mamba env create -f environment.yml
mamba activate Pi5ServoAICamera
```

### Pip/venv
```bash
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

## Hardware
- Raspberry Pi 5
- Pi Camera (Picamera2 compatible)
- 2x MG996R continuous rotation servo
- External servo power source

Pins (default):
- Left servo: GPIO 12
- Right servo: GPIO 13

## What Changed in This Refactor
- Removed contour-based fallback logic from active vision paths
- Removed HSV/threshold/morphology from active vision paths
- Standardized to Gaussian + Canny + Hough + PID
- Converted non-English reasoning text to English
- Slowed video processor playback significantly for demonstration
- Added richer debug overlays and telemetry focus on Hough coordinates

## Quick Presentation Script (for your boss)
1. Run `python video_processor.py`
2. Show the 3 windows and explain each stage:
   - Gray: intensity simplification
   - Blur: noise suppression
   - Canny: edge extraction with hysteresis
   - Hough: geometric line extraction
3. Explain lane center coordinate computation from left/right Hough groups
4. Explain PID terms (P and D) and final steering command
5. Show frame-by-frame logs and reasoning text in English

This gives a full, defensible engineering narrative from pixels to motor command.
