# Pi5ServoAICameraRectangular

A beginner-friendly line-following robot implementation for Raspberry Pi 5 using the Sony AI Camera (IMX500) and MG996R 360-degree servos.

## Features
- **Centering Logic**: Uses two-line recognition to keep the robot in the middle of a path.
- **Pi 5 Support**: Optimized for the Raspberry Pi 5 hardware, using `lgpio` and `gpiozero`.
- **AI Camera Integration**: Uses `Picamera2` to interface with the IMX500 sensor.
- **Conda Ready**: Includes `environment.yml` for easy setup with Miniforge or Anaconda.

## Hardware Requirements
- Raspberry Pi 5
- Raspberry Pi AI Camera (Sony IMX500)
- 2x MG996R 360-degree continuous rotation servos
- External 5V-6V power supply (for servos)
- Robot Chassis

## Setup

### Environment Installation (Miniforge/Mamba - Recommended)
Since you have Miniforge installed, use the faster `mamba` solver:
```bash
mamba env create -f environment.yml
mamba activate Pi5ServoAICamera
```

### Environment Installation (Conda Fallback)
If you prefer standard Conda:
```bash
conda env create -f environment.yml
conda activate Pi5ServoAICamera
```

### Environment Installation (Venv Fallback - Recommended for low RAM)
If Conda runs out of memory:
```bash
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
```

### Hardware Pins
- **Left Servo**: GPIO 12
- **Right Servo**: GPIO 13

## Project Structure (Digital Twin Architecture)
This project uses a unified backend logic that drives a virtual simulator, an offline video analyzer, and a physical robot identically.

- `robot_logic.py`: The core "Brain" (PD Controller, State Machine, Logging).
- `simulator.py`: A Pygame 2D simulator with virtual sensors.
- `video_processor.py`: OpenCV analysis of video footage.
- `main_pi.py`: Physical hardware execution running `RobotLogic`.
- `motor_control.py`: Servo signal management (MG996R 360).
- `environment.yml` / `requirements.txt`: Dependencies.

## How to Run

### 1. Run the Simulator (Virtual Testing) - WITH SLOW-MOTION DEBUGGING
```bash
python simulator.py
```
This opens a 2D Pygame window where a virtual car follows a track using the core PD logic.

**Features:**
- **2× Slow-Motion** enabled by default for detailed observation
- **Red detection areas** show where lines are found
- **Real-time PID dashboard** with algorithm reasoning
- **Pause Control (P key)** to stop and analyze specific moments
- Edit line 148 in simulator.py to change slow-motion factor:
  ```python
  main(slow_motion_factor=2)  # 1=realtime, 2=half-speed, 3=third-speed, etc.
  ```

### 2. Run Video Analyzer (Offline Video) - COMPLETE PIPELINE VISUALIZATION
```bash
python video_processor.py
```
Reads `test_run.mp4`, processes through complete algorithm pipeline with visualization.

**Features:**
- **Three synchronized display windows:**
  1. Main frame with PID logic overlay
  2. ROI detail with red detection areas and green center point
  3. **Algorithm pipeline stages** (Grayscale → Canny → HSV Mask → Clean Mask)
- **Slow-motion playback** (2× default, configurable)
- **Pause on P key**, resume with SPACE, quit with Q
- Edit line 194 in video_processor.py to customize:
  ```python
  process_video(
      debug_mode=True,              # Console debug info
      slow_motion_factor=2,         # Playback speed control
      show_pipeline=True            # Algorithm stages display
  )
  ```

### 3. Run on Physical Robot (Raspberry Pi 5)
```bash
python main_pi.py
```
Combines `Picamera2` live feed, MG996R servo control, and the shared `RobotLogic` to drive the physical robot.

**Enhanced Output:**
- Frame-by-frame status logging
- Line detection statistics (loss events, success rate)
- Comprehensive execution summary on shutdown

---

## 🔧 Debugging & Visualization Guide

For detailed debugging, see [DEBUG_GUIDE.md](DEBUG_GUIDE.md):
- Complete algorithm pipeline explanation
- Troubleshooting common issues
- Performance tuning tips
- Log file analysis
- **Understanding each processing stage with visual examples**

For comprehensive verification, see [VERIFICATION.md](VERIFICATION.md):
- Feature checklist for all components
- Digital twin architecture explanation
- Input/output specification verification
- Testing procedures (desktop and Pi)

---

## Algorithm Overview

The system uses a **Proportional-Derivative (PD) Controller** with two-line detection:

```
1. Detect two black lines (left and right borders)
2. Calculate midpoint between them
3. Calculate error = midpoint - image_center
4. Apply PD: output = Kp×error + Kd×(d_error/dt)
5. Command: left_motor = base_speed + output
            right_motor = -(base_speed - output)
```

**Slow-Motion Visualization Benefits:**
- Watch PD corrections in real-time
- See oscillation behavior clearly
- Understand error accumulation
- Tune Kp and Kd with confidence

---


