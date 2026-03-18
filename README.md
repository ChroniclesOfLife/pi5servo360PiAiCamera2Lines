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

### 1. Run the Simulator (Virtual Testing)
```bash
python simulator.py
```
This opens a 2D Pygame window where a virtual car follows a track using the core PD logic. Useful for tuning `Kp` and `Kd` safely.

### 2. Run Video Analyzer (Offline Video)
```bash
python video_processor.py
```
Reads `test_run.mp4`, processes the lines via OpenCV HSV, applies PD logic, and overlays the PID output and State on the video frames.

### 3. Run on Physical Robot (Raspberry Pi 5)
```bash
python main_pi.py
```
Combines `Picamera2` live feed, MG996R servo control, and the shared `RobotLogic` to drive the physical robot.

## License
MIT
