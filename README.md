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

### Environment Installation (Conda/Miniforge)
If your system has enough RAM:
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

## How to Run
```bash
python main.py
```

## Project Structure
- `main.py`: Entry point and P-control logic.
- `vision.py`: Image processing and line detection.
- `motor_control.py`: Servo signal management.
- `environment.yml`: Dependency list.

## License
MIT
