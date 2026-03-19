# Quick Start - Enhanced Debugging System

## 🚀 Get Started in 2 Minutes

### Option 1: Desktop Demo (No Hardware Needed)

**Show the algorithm in action:**
```bash
python video_processor.py
```
- **3 windows open** showing algorithm pipeline
- **Red areas** = detected black lines
- **Green dots** = calculated center point
- Press **P** to pause, **SPACE** to resume, **Q** to quit

### Option 2: Virtual Robot Simulator

**Watch the robot follow a virtual track:**
```bash
python simulator.py
```
- **Left side:** Track with red car
- **Right side:** PID algorithm dashboard
- **Press P** to pause and analyze
- **2× slow-motion** enabled by default for observation

### Option 3: Physical Robot (Pi 5 Only)

**Run on real hardware:**
```bash
python main_pi.py
```
- Same algorithm as simulator
- Reads actual camera input
- Controls real servos
- Logs all data for analysis

---

## 📊 What You'll See

### Video Processor - 3 Windows

**Window 1: Main Frame with Dashboard**
```
Frame: 125 | State: FOLLOWING | Error: -12.5px
ROI: [288-432] | Detections: 2 blob(s)
PID Output (Steering): -0.063
Cảm biến thấy line ở TRÁI xe (Error âm).
-> QUYẾT ĐỊNH: Rẽ Trái Nhẹ.
```

**Window 2: ROI Detail**
```
Shows zoomed region with:
- Green circle = detected center
- Red outline = detection boundary
- Red line = error from center
```

**Window 3: Pipeline Stages**
```
4 panels side-by-side:
1. Grayscale (from RGB)
2. Canny Edges (50-150 threshold)
3. HSV Mask (black color detection)
4. Clean Mask (after morphology)
```

### Simulator - 2 Sections

**Left: Pygame Track**
```
- Black rectangles = race track
- Red box = car position
- Yellow arrow = car heading
- Green dot = detected line center
- Cyan line = virtual camera scan
```

**Right: Dashboard**
```
Frame: 45
State: FOLLOWING
Detection: ✓ YES
Angle: 25.3°
Error: -8.5px
P_term: -0.045
D_term: -0.002
Total PID: -0.047
Reasoning: [algorithm decision]
```

---

## 🎮 Keyboard Controls

### Video Processor & Simulator
| Key | Action |
|-----|--------|
| **P** | Pause playback |
| **SPACE** | Resume (if paused) |
| **Q** | Quit application |
| **ESC** | Quit (Simulator only) |

---

## 🔧 Common Tasks

### Slow Down Playback (Better Observation)

**Video Processor:**
Edit line 194:
```python
process_video(slow_motion_factor=3)  # 1/3 speed
```

**Simulator:**
Edit line 148:
```python
main(slow_motion_factor=4)  # 1/4 speed
```

### Tune PID Controller

**Edit robot_logic.py (line 5):**
```python
logic = RobotLogic(kp=0.005, kd=0.001)
```

Try different values:
- ⬆️ Higher Kp → Stronger response (may oscillate)
- ⬇️ Lower Kp → Weaker response (slow to correct)
- ⬆️ Higher Kd → Smoother (may overshoot)
- ⬇️ Lower Kd → Faster (may oscillate)

### View Algorithm Stages

**From Python:**
```python
from vision import LineFollowerVision

vision = LineFollowerVision(debug_mode=True)
error, stages = vision.get_line_error(return_intermediate=True)

# Access any stage:
print(stages['gray'])          # Grayscale image
print(stages['canny'])         # Canny edges
print(stages['contour_count']) # Number of blobs found
```

---

## 📈 Understanding the Algorithm

### The PD Controller Output

```
Error = Detected_Center - Image_Center
  (negative = left, positive = right)

Output = (Kp × Error) + (Kd × ΔError)
         └─ Position │ └─ Momentum
                     
Positive Output → Turn RIGHT
Negative Output → Turn LEFT
```

### Two-Line Detection

```
Frame Input
    ↓
Find two largest black blobs
    ↓
Calculate their centers (cx1, cx2)
    ↓
Midpoint = (cx1 + cx2) / 2
    ↓
Error = Midpoint - Image_Center
    ↓
Feed to PD Controller
```

---

## 📋 Troubleshooting

### "Line Lost" Frequently

**Run video processor:**
```bash
python video_processor.py
```
Look at "Window 3: Pipeline Stages" - Is the black line visible in the "Clean Mask" panel?
- **Yes** → Image processing is working, check camera angle
- **No** → Adjust HSV mask bounds or morphological operations

### Robot Oscillates/Jerks

**Check PID dashboard:**
- Is D_term very large? → Lower Kd
- Is Error changing wildly? → Improve camera or lighting

### Reading Log Files

```bash
# See what happened during run:
head -20 log_video.txt    # First 20 frames
tail -20 log_video.txt    # Last 20 frames

# Analyze with Python:
import pandas as pd
df = pd.read_csv('log_video.txt')
df[['Frame', 'Error', 'P_Term', 'D_Term']].head(50)
```

---

## 📚 Complete Guides

For detailed information:

- **DEBUG_GUIDE.md** - Complete algorithm walkthrough
- **VERIFICATION.md** - Feature checklist and testing procedures
- **CHANGES_SUMMARY.md** - All modifications made
- **README.md** - Project overview

---

## 🎯 5-Minute Demo Script

**For showcasing the system:**

1. **(0-1 min) Hardware Overview**
   - Point out: Pi5, IMX500 camera, MG996R servos, nested rectangle track

2. **(1-3 min) Algorithm Pipeline**
   ```bash
   python video_processor.py
   ```
   - Play 10 seconds at 2× slow
   - Pause and show each window
   - Point out: "See the red detection area here"

3. **(3-5 min) Virtual Robot**
   ```bash
   python simulator.py
   ```
   - Let it run 30 seconds
   - Point out: "Same algorithm, different visualizer"
   - Press P to pause at interesting moment

**Total: 5 minutes, complete understanding**

---

## ✅ Verification Checklist

Before running on robot, verify on desktop:

- [ ] `python video_processor.py` opens 3 windows
- [ ] **Red areas** appear in window 2 when lines detected
- [ ] **Pipeline stages** visible in window 3 (4 panels)
- [ ] `python simulator.py` shows car following track
- [ ] **Dashboard** updates smoothly with PID values
- [ ] **Pause works** (P key stops animation)
- [ ] Log files created: `log_video.txt`, `log_robot.txt`

If all check, safe to deploy on Pi! ✅

---

## 🚨 Safety Notes

**On Physical Robot:**
- 3-second startup delay (safety)
- Robot stops if line is lost
- Always have emergency stop ready
- Test in controlled area first

---

## 📞 Quick Reference

| Goal | Command | Result |
|------|---------|--------|
| See algorithm stages | `python video_processor.py` | 3-window visualization |
| Debug line detection | Watch "Window 2: ROI Detail" | See red detection areas |
| Test PID tuning | `python simulator.py` | Virtual robot with dashboard |
| Understand reasoning | Check console output | Vietnamese algorithm logic |
| Analyze performance | View `log_*.txt` files | CSV with all data |
| Run robot | `python main_pi.py` | Physical hardware execution |

---

**Ready to start? Pick a mode above and launch!** 🚀

For questions, see DEBUG_GUIDE.md or VERIFICATION.md.

*Last updated: 2026-03-18*
