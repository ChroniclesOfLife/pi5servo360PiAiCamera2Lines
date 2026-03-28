# Pi5 Servo 360 + AI Camera 2-Line Robot

Production and demo repository for a Raspberry Pi 5 line-following robot using MG996R continuous servos and a downward-facing camera.

## Owner And Algorithm Attribution

This statement is included exactly as requested for presentation context:

- I understand all math.
- I understand the Pygame structure, but I did not implement that part myself.
- I understand all PID robot logic and related systems.
- The main algorithm idea is by me.

## Active Algorithm Stack (Current Version)

The active pipeline is strict and consistent across runtime/debug modules:

1. ROI crop
2. Grayscale conversion
3. Gaussian blur
4. Canny edge detection (hysteresis thresholds)
5. Probabilistic Hough transform
6. Lane center coordinate estimation from left/right line groups
7. PID steering control (PD form currently used: P + D)

Not used in active path: HSV masking, threshold segmentation, contour tracking, morphology.

## Repository Structure (GitHub Presentable)

```text
pi5servo360PiAiCamera2Lines/
|- README.md
|- LICENSE
|- requirements.txt
|- environment.yml
|- main_pi.py
|- main.py
|- vision.py
|- video_processor.py
|- robot_logic.py
|- motor_control.py
|- simulator.py
|- robot_brain.py
|- test_robot_brain_acceptance.py
|- test_run.mp4
|- docs/
|  |- QUICK_START.md
|  |- DEBUG_GUIDE.md
|  |- VERIFICATION.md
|  |- CHANGES_SUMMARY.md
|  |- FILE_CHANGES.md
|  |- CODE_REVIEW.md
|  `- COMPLETION_SUMMARY.txt
`- versions/
   |- README.md
   |- v1_legacy_f7da005/
   |  |- README.md
   |  |- main_pi.py
   |  |- vision.py
   |  |- video_processor.py
   |  |- robot_logic.py
   |  `- simulator.py
   `- v2_current/
      `- README.md
```

## Versioning

- Current active implementation: repository root source files + `versions/v2_current/`.
- Earlier archived snapshot: `versions/v1_legacy_f7da005/` (from commit `f7da005`).

This gives a clean old-vs-new narrative for management and technical review.

## Core Math (Quick Presentation Format)

Given lane-center error in pixels at frame $t$:

- $e_t = x_{lane,t} - x_{image-center}$
- $P_t = K_p \cdot e_t$
- $D_t = K_d \cdot (e_t - e_{t-1})$
- $u_t = P_t + D_t$

Motor mapping:

- `left_speed = BASE_SPEED + clamp(u_t)`
- `right_speed = -(BASE_SPEED - clamp(u_t))`

## ROI And Zone Logic (High Level)

- ROI is configured to the lower section of the frame where lane lines are closest and most stable.
- Hough segments are split into left/right groups using x-midpoints.
- Lane center is the average of left/right group means.
- Steering error is lane-center minus image-center.

State transition support exists in `robot_logic.py` (`FOLLOWING` and `TURNING_90`) and can be driven by trigger booleans from perception.

## How To Run

### 1) Physical Robot Runtime

```bash
python main_pi.py
```

### 2) Offline Debug + Presentation (Very Slow Playback)

```bash
python video_processor.py
```

Windows shown:

- Main dashboard with PID + state
- ROI overlay with Hough lines and lane center
- Pipeline view: Gray | Blur | Canny | Hough

Controls:

- `q` quit
- `p` pause
- `space` resume

### 3) Simulator

```bash
python simulator.py
```

## Documentation Map

- Quick start: `docs/QUICK_START.md`
- Debug details: `docs/DEBUG_GUIDE.md`
- Verification checklist: `docs/VERIFICATION.md`
- Changes and review artifacts: `docs/`

## Setup

### Conda

```bash
mamba env create -f environment.yml
mamba activate Pi5ServoAICamera
```

### venv

```bash
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

## Hardware Target

- Raspberry Pi 5
- Picamera2-compatible module
- 2x MG996R continuous servos
- External servo power

Default pins:

- Left: GPIO 12
- Right: GPIO 13

## Executive Summary (For Boss Presentation)

This repository now has:

- Clear and versioned structure
- Archived earlier implementation snapshot
- Current strict vision/control stack (Gaussian + Canny + Hough + PID)
- English debug reasoning and presentation-ready documentation
