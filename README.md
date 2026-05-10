![ARIA](cataria.png)
![ARIA](OBJ.png)

---

## Demo 1 — Encoders, IMU, Vacuuming and Trajectory

This demo showcases ARIA's core capabilities running live on the Arduino UNO Q:

- 📐 **Encoders & IMU** — real-time wheel odometry and inertial measurement feeding the EKF pose estimator
- 🗺️ **Trajectory** — the robot following a path while the occupancy grid updates live in the web dashboard
- 🌀 **Vacuum control** — the L298N-driven vacuum motor being toggled and speed-adjusted via the web UI and Telegram bot
- 🔍 **Object detection** — the `VideoObjectDetection` brick identifying objects through the camera stream

<video controls width="100%" src="output.mp4">
  Your browser does not support the video tag.
  <a href="output.mp4">Download Demo 1 (output.mp4)</a>
</video>

> **Demo1** — recorded on the Arduino UNO Q running ARIA-step-four.
