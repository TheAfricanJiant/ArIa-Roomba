# ARIA — Prioritized Build Plan

> Build in order. Don't skip ahead. A working robot at phase 3 beats a broken robot at phase 6.

---

## PHASE 1 — Core Hardware & Real-Time MCU (Week 1–2) ✅ IMPLEMENTED
*Nothing else works until this is solid.*

- [x] Assemble chassis — motors, encoders, wheels, vacuum mount, brush roller (Using XRP Platform)
- [x] Wire motors to XRP motor driver (DRV8835)
- [x] Wire 6× HC-SR04 ultrasonics (front, F-L, F-R, left, right, rear)
- [x] Access onboard LSM6DSOX IMU via I²C on XRP
- [x] Power rail: separate motor power from logic power (handled by XRP power circuit)
- [x] Flash XRP sketch via PlatformIO (see XRW folder):
  - [x] Motor PWM output (DRV8835 EN/PH, -255 to 255 per wheel)
  - [x] Encoder ISR (`attachInterrupt()`, quadrature counting)
  - [x] Ultrasonic async polling (all 6, full cycle every 50 ms)
  - [x] IMU I²C read (LSM6DSOX gyro + accel at 104 Hz)
  - [x] Complementary filter pre-smoothing on XRP
- [x] Verify motors spin correctly in both directions
- [x] Verify encoder tick counts increase when wheels turn
- [x] Verify ultrasonic readings are sane (no noise, no stuck values)
- [x] Verify IMU yaw changes when robot rotates

---

## PHASE 2 — App Lab Bridge & Python Comms (Week 2)
*Get STM32 ↔ Linux talking reliably before writing any AI code.*

- [ ] Set up App Lab Bridge serial protocol:
  - [ ] Uplink packet: `S,u1..6,encL,encR,gz,ts\n` every 50 ms
  - [ ] Downlink: `M,lPWM,rPWM\n` · `V,vac\n` · `B,brush\n`
- [ ] Python script to receive and parse sensor packets
- [ ] Python script to send motor commands and verify wheel movement
- [ ] Latency test: measure round-trip command → response time
- [ ] Stress test: 5-minute continuous run, check for dropped packets

---

## PHASE 3 — EKF Localization & Occupancy Grid ✅ IMPLEMENTED
*The brain. Without this, there is no navigation.*

- [x] Install `filterpy` on UNO Q Linux (`pip install -r python/requirements.txt`)
- [x] Implement EKF `predict(enc_left_delta, enc_right_delta)` — dead-reckoning
- [x] Implement EKF `correct_imu(gyro_z, dt)` — heading correction (Joseph-form update)
- [x] Implement `wall_snap(side, wall_coord_cm)` — drift reset from side ultrasonics
- [x] Implement `OccupancyGrid` class (30 cm cells, uint8 NumPy array)
- [x] `mark_cleaned(x, y)` — mark cells as CLEANED as robot moves
- [x] `update_from_ultrasonics(x, y, theta, distances)` — ray-march to mark WALL/FREE
- [x] `coverage_percent()` — live coverage readout
- [x] `nearest_uncleaned(x, y)` — vectorised search for INSPECT state
- [x] `nav_array` property — boolean passable array for A* (Phase 5)
- [x] `save()` / `load()` — persistent grid across sessions
- [x] ASCII terminal visualisation (`print_terminal`)
- [x] `BridgeStub` simulator — test without any hardware
- [x] `aria_main.py` — 20 Hz rate-limited loop, argparse, graceful shutdown
- [ ] Run simulator and verify map builds correctly → `python aria_main.py`
- [ ] Connect real hardware and verify live map → `python aria_main.py --port /dev/ttyUSB0`

---

## PHASE 4 — Navigation: Boustrophedon Coverage ✅ IMPLEMENTED
*The lawnmower. This is the core cleaning behaviour.*

- [x] Implement `BoustrophedonPlanner` class (`aria/navigation.py`)
  - [x] `next_waypoint()` — stripe-based forward/backward passes
  - [x] Stripe shifting (one robot-width, reverse direction)
  - [x] `is_complete()` — coverage ≥ 95% check
  - [x] `_find_stripe_end()` — scans grid to find wall/obstacle limit per stripe
- [x] Implement `PotentialFieldSteering` — smooth real-time avoidance
  - [x] Attractive force toward waypoint (proportional to distance)
  - [x] Repulsive forces from all 6 ultrasonic sensors (< 50 cm)
  - [x] Speed scaling (slows near waypoint)
  - [x] Output: `(left_pwm, right_pwm)` differential drive
- [x] Implement `CleaningStateMachine` — 6 states
  - [x] IDLE → NAV → CLEAN → AVOID → INSPECT → DOCK
  - [x] Actuator PWM (vacuum + brush) per state
  - [x] Obstacle stop + AVOID timeout with rotation escape
  - [x] Low-battery → DOCK trigger (Phase 5 will add real ADC)
  - [x] INSPECT: `nearest_uncleaned()` to mop missed zones
- [x] Integrated into `main.py` nav loop (step() called every 50 ms)
- [x] WebUI handlers: `start_clean`, `stop_clean`, `get_state`
- [x] `cleaner.state_name` pushed to UI in every `nav_state` message
- [ ] Test: robot cleans a 2 m × 2 m area with no furniture
- [ ] Test: robot navigates around a cardboard box obstacle
- [ ] Test: coverage % reaches > 90% before stopping

---

## PHASE 5 — A* Path Planning & Auto-Dock ✅ IMPLEMENTED
*Return to charge. This closes the autonomous loop.*

- [x] Implement A* on occupancy grid (`heapq` based) → `aria/astar.py`
  - [x] `astar_cells(passable, start, goal)` — 8-directional, Euclidean heuristic
  - [x] Goal snapping — if goal cell is blocked, snaps to nearest passable (3-cell radius)
  - [x] `_simplify_path()` — removes collinear waypoints (~60–80% reduction)
  - [x] `plan_path(grid, sx, sy, gx, gy)` — world-coordinate wrapper (cm ↔ cells)
- [x] Record dock station position
  - [x] `on_set_dock` — explicit (x, y) or `{ here: true }` to use current pose
  - [x] `cleaner.set_dock_position(x, y)` — persists in session
- [x] DOCK state in the 6-state machine (fully A* driven)
  - [x] Trigger: `battery_pct < 15%` → `CleanState.DOCK`
  - [x] A* path planned on first tick in DOCK, replanned if needed
  - [x] Fallback: direct proportional drive if A* finds no path
  - [x] Arrived → `CleanState.IDLE`
- [x] `on_trigger_dock` — WebUI manual dock command
- [x] `_battery_pct` global wired into `cleaner.step()` (replaces hardcoded 100.0)
- [x] `on_set_battery` — WebUI battery override for testing dock trigger
- [x] INSPECT state upgraded — A* path to each missed zone
  - [x] Replan only when path exhausted (prevents thrashing)
  - [x] Loops until `nearest_uncleaned()` returns None → DOCK
- [x] `on_plan_path_debug` — sends A* path as JSON to WebUI for visualisation
- [x] `plan_path` / `astar_cells` exported from `aria` package
- [ ] Test: robot docks reliably from 3 different starting positions
- [ ] End-to-end test: full autonomous clean → auto-dock

---

## PHASE 6 — Nano BLE Sense Audio Model (Week 6)
*The unique feature. Train, deploy, integrate.*

- [ ] Record training audio (vacuum running):
  - [ ] `clear` — empty floor (2 min minimum)
  - [ ] `dust` — fine dust picked up (2 min)
  - [ ] `crumbs` — food crumbs (2 min)
  - [ ] `gravel` — grit / larger debris (2 min)
- [ ] Upload dataset to Edge Impulse Studio
- [ ] Train MFE Spectrogram + NN model
- [ ] Validate accuracy > 85% before deploying
- [ ] Export as Arduino library, flash to Nano 33 BLE Sense
- [ ] Wire Nano to UNO Q:
  - [ ] Option A: UART (Nano TX → UNO Q RX)
  - [ ] Option B: BLE notify characteristic
- [ ] Python receiver: parse `D,class,confidence\n` on UNO Q
- [ ] Feed into `DirtHeatmap.log_debris(ekf_x, ekf_y, class, conf)`
- [ ] Adjust brush PWM in real-time from debris class
- [ ] Adjust vacuum PWM from debris confidence score
- [ ] After 5+ cleaning runs: export heatmap PNG with `matplotlib`

---

## PHASE 7 — Web UI Dashboard (Week 7)
*The face of the project. Judges will see this.*

- [ ] Live occupancy grid map rendered in browser (Canvas / SVG)
- [ ] Live sensor readouts panel (ultrasonic distances, battery %, coverage %)
- [ ] Live EKF position dot overlaid on map
- [ ] Dirt heatmap overlay toggle (show/hide)
- [ ] Coverage percentage progress bar
- [ ] Robot status indicator (IDLE / NAV / CLEAN / AVOID / DOCK)
- [ ] **Manual control panel** (see Phase 8)
- [ ] Clean, mobile-friendly layout using existing Arduino CSS theme

---

## PHASE 8 — Manual Control & Camera on Web UI (Week 7–8)
*Lets you drive, take pictures, record video — all from the browser.*

- [ ] D-pad / joystick control (WASD or on-screen buttons)
  - [ ] Sends `M,lPWM,rPWM` to STM32 via Bridge on keypress/hold
  - [ ] Release = stop
- [ ] Vacuum toggle button (on/off)
- [ ] Brush speed slider
- [ ] Live camera stream (MJPEG over WebSocket from UNO Q OpenCV)
- [ ] **Take Snapshot** button → saves JPEG + timestamp + EKF position
- [ ] **Start/Stop Recording** button → records video + audio from mic
  - [ ] Saves `.mp4` to `/home/arduino/recordings/`
- [ ] **Download last recording** link
- [ ] Keyboard shortcut overlay (for demo day)

---

## PHASE 9 — Telegram Bot Integration (Week 8)
*Remote control and alerts from anywhere.*

- [ ] Set up Telegram bot via BotFather, store token securely
- [ ] Install `python-telegram-bot` on UNO Q
- [ ] Implement bot commands:
  - [ ] `/start` — welcome + status summary
  - [ ] `/status` — battery %, coverage %, current state, position
  - [ ] `/clean` — start autonomous cleaning run
  - [ ] `/stop` — stop robot immediately
  - [ ] `/dock` — send robot to dock now
  - [ ] `/photo` — capture snapshot and send to Telegram chat
  - [ ] `/video [seconds]` — record N seconds, send clip
  - [ ] `/map` — send current occupancy map as image
  - [ ] `/heatmap` — send current dirt heatmap PNG
  - [ ] `/summary` — cleaning run summary (time, coverage %, zones found dirty)
  - [ ] `/alerts on/off` — toggle automatic obstacle/event alerts
- [ ] Automatic alerts (push to Telegram without prompting):
  - [ ] Pet/person detected → photo + message
  - [ ] Obstacle blocked path → location + photo
  - [ ] Battery low (< 20%) → alert
  - [ ] Cleaning complete → summary + map
  - [ ] Robot stuck / error → alert + position
- [ ] Security: whitelist allowed Telegram user IDs (no public access)

---

## PHASE 10 — Obstacle Classifier / Camera AI (Week 9)
*Add vision-based obstacle detection — pets, people, furniture.*

- [ ] Confirm USB camera appears as `/dev/video0` (UVC)
- [ ] Collect FOMO training dataset:
  - [ ] `pet` — cats/dogs on floor (100+ images, varied angles)
  - [ ] `person` — feet/legs visible (100+ images)
  - [ ] `furniture` — chair/table legs (100+ images)
  - [ ] `toy` — toys on floor (100+ images)
  - [ ] Use Roboflow Universe to supplement
- [ ] Train FOMO model on Edge Impulse
- [ ] Deploy as `.eim` to UNO Q Linux
- [ ] Implement camera capture thread (OpenCV, 96×96 for AI, 320×240 for stream)
- [ ] Integrate with cleaning logic:
  - [ ] `pet` / `person` → AVOID state, full stop, 5s wait, send Telegram alert + photo
  - [ ] `furniture` → potential field reroute
  - [ ] `toy` → reroute + log to dashboard
- [ ] Test in real room with real obstacles

---

## PHASE 11 — Polish, Testing & Competition Prep (Week 10)
*Make it reliable. Make it presentable. Win.*

- [ ] Pre-load 5 runs of heatmap data so prediction is visible on demo day
- [ ] End-to-end autonomous run: clean → dock → send Telegram summary
- [ ] Record a full demo video (failsafe if live demo has issues)
- [ ] Competition pitch (2-minute version):
  - [ ] "ARIA is the only Roomba that knows where dirt will be before it cleans"
  - [ ] Show audio heatmap live
  - [ ] Show Telegram alert with pet photo
  - [ ] Show manual control + live stream
- [ ] Stress test: 3 consecutive autonomous runs, no crashes
- [ ] BLE fallback: have UART cable ready if BLE drops in venue
- [ ] Battery runtime test: confirm 3+ hour runtime
- [ ] Backup: have pre-recorded video of every feature working

---

## Feature Summary

| Feature | Phase | Status |
|---------|-------|--------|
| Motor + encoder control | 1 | ⬜ |
| Serial Bridge comms | 2 | ⬜ |
| EKF localization | 3 | ⬜ |
| Occupancy grid | 3 | ⬜ |
| Boustrophedon coverage | 4 | ⬜ |
| Potential field avoidance | 4 | ⬜ |
| A* path planning + dock | 5 | ⬜ |
| Nano audio EI model | 6 | ⬜ |
| Dirt heatmap | 6 | ⬜ |
| Web UI dashboard + map | 7 | ⬜ |
| Manual control + camera | 8 | ⬜ |
| Telegram bot | 9 | ⬜ |
| FOMO obstacle classifier | 10 | ⬜ |
| Competition polish | 11 | ⬜ |

> ⬜ Not started · 🔄 In progress · ✅ Done
