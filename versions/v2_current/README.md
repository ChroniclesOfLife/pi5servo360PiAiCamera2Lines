# v2 Current (Gaussian + Canny + Hough + PID)

This is the current active architecture represented by the repository root source files.

Active pipeline:
1. ROI crop
2. Grayscale
3. Gaussian blur
4. Canny edge detection (hysteresis thresholds)
5. Hough line transform
6. Lane center coordinate estimate
7. PID steering control

The latest documentation is in `docs/` and the top-level `README.md`.
