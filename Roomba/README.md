# ARIA — Autonomous Roomba with Intelligent Awareness

**ARIA** is an autonomous cleaning robot powered by the **Arduino UNO Q**, leveraging its dual-processor architecture (Qualcomm Dragonwing QRB2210 Linux MPU + STM32U585 real-time MCU) to deliver intelligent, AI-driven floor cleaning. An **Arduino Nano 33 BLE Sense** serves as a dedicated audio co-processor, running Edge Impulse debris classification and streaming results to the UNO Q.

![ARIA System Architecture](assets/docs_assets/thumbnail.png)

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Bricks Used](#bricks-used)
- [Sensor Configuration](#sensor-configuration)
- [Edge Impulse AI Models](#edge-impulse-ai-models)
- [Navigation, Mapping & Path Planning](#navigation-mapping--path-planning)
- [Telegram Bot Integration](#telegram-bot-integration)
- [Manual Control & Live Camera](#manual-control--live-camera)
- [Data Flow](#data-flow)
- [How to Use](#how-to-use)
- [Understanding the Code](#understanding-the-code)
- [Power Budget](#power-budget)
- [Development Timeline](#development-timeline)

---

## Overview

ARIA splits intelligence across **three processors**:

| Layer | Processor | Role |
|-------|-----------|------|
| **Real-time control** | STM32U585 MCU (on UNO Q) | Motor PWM, encoder ISR, ultrasonic polling, IMU I²C, serial protocol |
| **Linux intelligence** | Qualcomm MPU (on UNO Q, 2–4 GB RAM) | EKF localization, coverage navigation, cleaning logic, camera AI, heatmap logging |
| **Audio AI co-processor** | Arduino Nano 33 BLE Sense | Edge Impulse audio debris classification → sends results to UNO Q via BLE/Serial |

The **App Lab Bridge** connects the UNO Q's two sides over bidirectional USB serial at 115200 baud — sensor packets flow upward every 50 ms, motor commands flow downward every 20 ms. The **Nano BLE Sense** communicates classification results to the UNO Q over BLE or a dedicated UART line.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  HARDWARE LAYER — Physical World                            │
│  USB Camera · 6× Ultrasonic (HC-SR04) · MPU6050 IMU         │
│  Encoder Motors ×2 · Vacuum Motor · Brush Roller            │
└────────────────────────┬────────────────────────────────────┘
                         │ GPIO · I²C · PWM · UART
┌────────────────────────▼────────────────────────────────────┐
│  STM32 — Arduino C++ (Real-time control)                    │
│  Motor PWM · Encoder ISR · Ultrasonic Poll · IMU I²C        │
│  Serial Protocol (ASCII CSV @ 115200 baud)                  │
└────────────────────────┬────────────────────────────────────┘
                         │ App Lab Bridge
                         │ ↑ sensor data (50 ms)
                         │ ↓ commands    (20 ms)
┌────────────────────────▼────────────────────────────────────┐
│  PYTHON — Linux Intelligence (Qualcomm MPU)                 │
│  EKF Localization · Coverage Navigator · Cleaning Logic     │
│  Camera Handler · Heatmap Logger                            │
│                                                             │
│  ◄──── BLE / UART ────► ┌─────────────────────────────┐    │
│   debris class + conf   │ ARDUINO NANO 33 BLE SENSE   │    │
│                          │ Edge Impulse Audio Debris   │    │
│                          │ MFE Spectrogram NN          │    │
│                          │ Onboard PDM Microphone      │    │
│                          └─────────────────────────────┘    │
└────────────────────────┬────────────────────────────────────┘
                         │ inference calls · results
┌────────────────────────▼────────────────────────────────────┐
│  EDGE IMPULSE AI MODELS                                     │
│  Floor Classifier (UNO Q GPU) · Obstacle Classifier (FOMO)  │
│  Audio Debris (Nano BLE Sense — dedicated co-processor)      │
└────────────────────────┬────────────────────────────────────┘
                         │ classification → action decisions
┌────────────────────────▼────────────────────────────────────┐
│  OUTPUTS & ACTIONS                                          │
│  Motor Commands · Suction Adjust · Brush Speed              │
│  Dirt Heatmap · Obstacle Alert                              │
└─────────────────────────────────────────────────────────────┘
```

> **Full interactive architecture diagram** → open [`aria_full_system_architecture.html`](aria_full_system_architecture.html) in a browser to tap any component for implementation details.

---

## Hardware Requirements

| Component | Specification | Purpose | Est. Cost |
|-----------|--------------|---------|-----------|
| Arduino UNO Q | Qualcomm QRB2210 + STM32U585 | Main board (Linux + real-time) | — |
| **Arduino Nano 33 BLE Sense** | **nRF52840 + onboard PDM mic** | **Dedicated audio AI co-processor** | **~$33** |
| USB-C to USB-A Cable | ×1 | Programming / power | — |
| HC-SR04 Ultrasonic | ×6 (front, front-L, front-R, left, right, rear) | Wall following + obstacle avoidance | ~$6 |
| MPU6050 IMU | ×1 (I²C) | Heading correction via EKF | ~$3 |
| Encoder DC Motors | ×2 with quadrature encoders | Differential drive + odometry | — |
| TB6612FNG Motor Driver | ×1 | Motor PWM control | ~$4 |
| Vacuum Motor | 775 DC or equivalent | Suction (PWM controlled) | ~$8 |
| Brush Roller Motor | N20 DC motor | Front brush roller | ~$3 |
| USB Camera | UVC compatible (e.g., Anycubic Kobra 3) | Floor/obstacle classification | — |

### Optional

- USB-C Hub with HDMI for SBC mode (monitor + keyboard + mouse)
- 3D-printed brush housing (Mark II design)

---

## Software Requirements

- **Arduino App Lab** — unified IDE for Python + C++ development
- **Arduino IDE 2.x** — for flashing the Nano 33 BLE Sense
- **Edge Impulse Linux SDK** — for on-device AI inference (UNO Q)
- **Edge Impulse Arduino library** — deployed model for Nano BLE Sense
- **Python libraries** (Linux side): `filterpy`, `opencv-python`, `numpy`, `matplotlib`, `Pillow`
- **Arduino libraries** (STM32 side): `MPU6050.h`, `TB6612FNG`
- **Arduino libraries** (Nano BLE Sense): `ArduinoBLE.h`, `PDM.h`, Edge Impulse SDK

---

## Bricks Used

| Brick | Purpose |
|-------|---------|
| `web_ui` | Web interface — dashboard, map, manual control |
| `object_detection` | Base object detection (extended for FOMO obstacle classifier) |
| `telegram_bot` | Remote control, alerts, photo/video sharing via Telegram |

---

## Sensor Configuration

### 6× Ultrasonic Ring (HC-SR04)

```
          [Front]
      [F-Left] [F-Right]
  [Left]    ARIA    [Right]
          [Rear]
```

- **Range**: 2–400 cm
- **Full cycle rate**: All 6 sensors every 50 ms
- **Front sensors** → obstacle stop trigger (< 15 cm)
- **Side sensors** → wall-following + EKF drift correction (< 8 cm = wall boundary)

### MPU6050 IMU

- **Interface**: I²C on STM32
- **Data**: Gyroscope Z-axis (yaw rate) + Accelerometer X/Y
- **Rate**: 100 Hz with complementary pre-filter on STM32
- **Purpose**: Heading correction — without this, the robot curves off-track within metres

### Encoder Motors

- **Type**: Quadrature encoders, hardware interrupt (ISR)
- **Bridge output**: `encL`, `encR` tick deltas every 20 ms
- **Driver**: TB6612FNG (PWM range: -255 to 255 per wheel, 20 kHz)

---

## Edge Impulse AI Models

ARIA runs **two** Edge Impulse models. Audio classification is handled on the dedicated **Nano 33 BLE Sense**; vision classification on the **UNO Q GPU**.

### 1. Obstacle Classifier — Runs on UNO Q (FOMO)

| Property | Value |
|----------|-------|
| Model | FOMO (Faster Objects More Objects) |
| Classes | pet, person, furniture, toy |
| Inference rate | 30 fps real-time |
| Dataset | Roboflow Universe + custom |
| Output action | Living obstacles → full stop + 5s wait + reroute. Non-living → normal avoidance |

### 2. Audio Debris Model — Runs on Arduino Nano 33 BLE Sense

> **Why a dedicated board?** The Nano 33 BLE Sense has a built-in PDM microphone (MP34DT05) and enough power (nRF52840, 256 KB RAM) to run a small Edge Impulse audio model continuously at near-zero latency. This offloads audio processing entirely from the UNO Q, keeping the Qualcomm MPU free for navigation and vision.

| Property | Value |
|----------|-------|
| **Runs on** | Arduino Nano 33 BLE Sense (dedicated) |
| Microphone | Onboard MP34DT05 PDM (no external mic needed) |
| Model | MFE Spectrogram + Neural Network |
| Classes | dust, crumbs, gravel, clear |
| Training data | 2 min audio per class |
| Output | Sends `{class, confidence}` to UNO Q via BLE or UART |
| **Unique factor** | No competitor has acoustic debris classification |

#### Nano → UNO Q Communication

```
Nano 33 BLE Sense                    Arduino UNO Q (Python side)
┌──────────────┐    BLE / UART       ┌──────────────────────┐
│ PDM Mic      │───────────────────▶ │ Receive:             │
│ EI inference │  "D,crumbs,0.91"   │   debris_class       │
│ every 1s     │                     │   confidence         │
└──────────────┘                     │ → heatmap logger     │
                                     │ → brush speed adjust │
                                     └──────────────────────┘
```

**Option A — BLE** (wireless, cleanest wiring):
```cpp
// Nano BLE Sense side
BLEService debrisService("19B10000-...");
BLEStringCharacteristic debrisChar("19B10001-...", BLERead | BLENotify, 20);
// After EI inference:
debrisChar.writeValue("crumbs,0.91");
```

**Option B — UART** (simplest, most reliable):
```cpp
// Nano BLE Sense side — Serial1 TX → UNO Q RX
Serial1.println("D,crumbs,0.91");
```

---

## Navigation, Mapping & Path Planning

ARIA solves **three core problems** to clean autonomously:

| Problem | Question | How ARIA Solves It |
|---------|----------|-------------------|
| **Localization** | "Where am I?" | EKF fusing encoders + IMU + ultrasonic wall detection |
| **Mapping** | "What does the room look like?" | Occupancy grid built from ultrasonic readings |
| **Path Planning** | "Where do I go next?" | Boustrophedon coverage + A* for return-to-dock |

---

### 1. Localization — EKF (`filterpy.kalman.EKF`)

The **Extended Kalman Filter** fuses noisy sensor data into a stable position estimate:

- **State vector**: `[x, y, θ]` (position in cm + heading in radians)
- **Prediction step**: Encoder ticks → wheel velocities → dead-reckoning position update
- **Correction step**: IMU gyroscope Z corrects heading drift every 10 ms
- **Wall reset**: When side ultrasonic < 8 cm, the robot knows it's at a wall boundary — this resets accumulated drift

```python
from filterpy.kalman import ExtendedKalmanFilter

ekf = ExtendedKalmanFilter(dim_x=3, dim_z=2)  # [x, y, θ] state, [enc, imu] measurements

def predict_from_encoders(enc_left, enc_right):
    """Dead-reckoning: encoder ticks → distance → position update"""
    d_left  = enc_left  * WHEEL_CIRCUMFERENCE / TICKS_PER_REV
    d_right = enc_right * WHEEL_CIRCUMFERENCE / TICKS_PER_REV
    d_center = (d_left + d_right) / 2
    d_theta  = (d_right - d_left) / WHEEL_BASE
    
    ekf.x[0] += d_center * cos(ekf.x[2])  # x
    ekf.x[1] += d_center * sin(ekf.x[2])  # y
    ekf.x[2] += d_theta                    # θ

def correct_from_imu(gyro_z):
    """IMU heading correction — prevents drift"""
    ekf.update(z=[gyro_z], HJacobian=H_imu, hx=hx_imu)

def correct_from_wall(side_distance, wall_position):
    """Wall boundary reset — eliminates accumulated error"""
    if side_distance < 8:  # cm
        ekf.x[0] = wall_position  # snap to known wall
```

**Without EKF**: Robot drifts 5–10% per metre → unusable after 3 metres  
**With EKF**: Drift < 3% over entire cleaning run

---

### 2. Mapping — Occupancy Grid

The room is represented as a **grid of cells** (each 30 cm × 30 cm). Each cell has a state:

```
┌────┬────┬────┬────┬────┬────┬────┬────┐
│ W  │ W  │ W  │ W  │ W  │ W  │ W  │ W  │   W = Wall (blocked)
├────┼────┼────┼────┼────┼────┼────┼────┤   ✓ = Cleaned
│ W  │ ✓  │ ✓  │ ✓  │ ✓  │ ✓  │ ✓  │ W  │   · = Unvisited
├────┼────┼────┼────┼────┼────┼────┼────┤   ▓ = Obstacle
│ W  │ ✓  │ ✓  │ ▓  │ ▓  │ ✓  │ ✓  │ W  │   R = Robot
├────┼────┼────┼────┼────┼────┼────┼────┤
│ W  │ ✓  │ ✓  │ ▓  │ ▓  │ ·  │ ·  │ W  │
├────┼────┼────┼────┼────┼────┼────┼────┤
│ W  │ R→ │ ·  │ ·  │ ·  │ ·  │ ·  │ W  │
├────┼────┼────┼────┼────┼────┼────┼────┤
│ W  │ W  │ W  │ W  │ W  │ W  │ W  │ W  │
└────┴────┴────┴────┴────┴────┴────┴────┘
```

```python
import numpy as np

class OccupancyGrid:
    # Cell states
    UNKNOWN  = 0
    FREE     = 1   # Detected as open space
    CLEANED  = 2   # Robot has passed through
    WALL     = 3   # Ultrasonic detected wall
    OBSTACLE = 4   # Ultrasonic or AI detected obstacle
    
    def __init__(self, width_cm=500, height_cm=500, cell_size=30):
        cols = width_cm // cell_size
        rows = height_cm // cell_size
        self.grid = np.zeros((rows, cols), dtype=np.int8)
        self.cell_size = cell_size
    
    def update_from_position(self, x, y):
        """Mark current robot position as cleaned"""
        col, row = int(x / self.cell_size), int(y / self.cell_size)
        self.grid[row][col] = self.CLEANED
    
    def update_from_ultrasonic(self, robot_x, robot_y, robot_theta, distances):
        """Mark wall/obstacle cells from ultrasonic readings"""
        for angle_offset, dist in distances.items():
            if dist < 400:  # Valid reading
                wall_x = robot_x + dist * cos(robot_theta + angle_offset)
                wall_y = robot_y + dist * sin(robot_theta + angle_offset)
                col = int(wall_x / self.cell_size)
                row = int(wall_y / self.cell_size)
                self.grid[row][col] = self.WALL
    
    def coverage_percent(self):
        """How much of the known free space has been cleaned?"""
        cleanable = np.count_nonzero(self.grid >= self.FREE)
        cleaned   = np.count_nonzero(self.grid == self.CLEANED)
        return (cleaned / max(cleanable, 1)) * 100

    # Memory: 500cm × 500cm room @ 30cm cells = ~280 bytes!
```

---

### 3. Path Planning — Two Strategies

ARIA uses **two different path planners** for different situations:

#### Strategy A: Boustrophedon Coverage ("Lawnmower")

**Purpose**: Clean every cell in the room systematically  
**When**: Primary cleaning mode — this is how the room gets cleaned

```
Pass 1 →  ─────────────────────▶ │
                                  │ shift 1 robot-width
Pass 2 →  ◀───────────────────── │
          │
Pass 3 →  ─────────────────────▶ │
                                  │
Pass 4 →  ◀───────────────────── │
          │
Pass 5 →  ────────▶ OBSTACLE ──skip──▶ continue ──▶
```

```python
class BoustrophedonPlanner:
    def __init__(self, grid, robot_width_cm=30):
        self.grid = grid
        self.stripe_width = robot_width_cm
        self.current_stripe = 0
        self.direction = 1  # 1 = forward, -1 = backward
    
    def next_waypoint(self, robot_x, robot_y):
        """Returns the next (x, y) target for the robot"""
        stripe_y = self.current_stripe * self.stripe_width
        
        if self.direction == 1:
            # Scan forward until wall
            target_x = self.find_wall_in_direction(robot_x, stripe_y, +1)
        else:
            # Scan backward until wall
            target_x = self.find_wall_in_direction(robot_x, stripe_y, -1)
        
        # If we've reached the end of this stripe
        if abs(robot_x - target_x) < self.stripe_width:
            self.current_stripe += 1      # Move to next stripe
            self.direction *= -1           # Reverse direction
            return (robot_x, stripe_y + self.stripe_width)  # Shift over
        
        return (target_x, stripe_y)
    
    def is_complete(self):
        """Coverage check — are we done?"""
        return self.grid.coverage_percent() > 95
```

**Why Boustrophedon?** It guarantees 100% coverage of reachable space. Unlike random bounce (old Roombas), it never wastes time re-cleaning areas.

#### Strategy B: A* Path Planning (Point-to-Point)

**Purpose**: Navigate from current position to a specific target  
**When**: Return-to-dock, reach an uncleaned zone, navigate around large obstacles

```python
import heapq

def astar(grid, start, goal):
    """A* pathfinding on the occupancy grid"""
    rows, cols = grid.shape
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1]
        
        # Check 8 neighbours (including diagonals)
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            
            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue
            if grid[neighbor[0]][neighbor[1]] >= 3:  # WALL or OBSTACLE
                continue
            
            move_cost = 1.414 if (dx != 0 and dy != 0) else 1.0
            tentative_g = g_score[current] + move_cost
            
            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, neighbor))
    
    return None  # No path found

def heuristic(a, b):
    """Euclidean distance"""
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5
```

**Use cases for A***:

| Scenario | Start | Goal |
|----------|-------|------|
| Battery low (< 15%) | Current position | Docking station |
| Missed zone detected | Current position | Nearest uncleaned cell |
| Obstacle reroute | Before obstacle | First free cell past it |

---

### 4. Obstacle Avoidance — Potential Field Method

For **real-time** obstacle avoidance (while driving), ARIA uses a potential field:

- **Attractive force**: Pulls robot toward the next waypoint
- **Repulsive force**: Pushes robot away from ultrasonic-detected obstacles
- **Result**: Robot smoothly curves around obstacles instead of hard-stopping

```python
def potential_field_steering(robot_pos, waypoint, ultrasonic_readings):
    """Returns adjusted (left_pwm, right_pwm) for smooth avoidance"""
    # Attractive force toward waypoint
    dx = waypoint[0] - robot_pos[0]
    dy = waypoint[1] - robot_pos[1]
    attract_angle = atan2(dy, dx)
    
    # Repulsive forces from nearby obstacles
    repulse_x, repulse_y = 0, 0
    for sensor_angle, distance in ultrasonic_readings.items():
        if distance < 50:  # Only repel if < 50 cm
            strength = (50 - distance) / 50  # Stronger when closer
            repulse_x -= strength * cos(sensor_angle)
            repulse_y -= strength * sin(sensor_angle)
    
    # Combine forces
    final_angle = atan2(dy + repulse_y, dx + repulse_x)
    
    # Convert to differential drive PWM
    angle_error = final_angle - robot_heading
    base_speed = 180
    left_pwm  = base_speed - (angle_error * 50)
    right_pwm = base_speed + (angle_error * 50)
    
    return clamp(left_pwm, -255, 255), clamp(right_pwm, -255, 255)
```

---

### 5. Cleaning Logic — 6-State Machine

```
┌──────┐    ┌──────┐    ┌───────┐    ┌─────────┐    ┌──────┐    ┌──────┐
│ IDLE │───▶│ NAV  │───▶│ CLEAN │───▶│ INSPECT │───▶│ DOCK │───▶│ IDLE │
└──────┘    └──┬───┘    └───────┘    └─────────┘    └──────┘    └──────┘
               │
               ▼
          ┌─────────┐
          │  AVOID  │ (obstacle detected)
          └─────────┘
```

| State | Trigger In | Action | Trigger Out |
|-------|-----------|--------|-------------|
| **IDLE** | Power on / dock complete | Wait for start command | User starts clean |
| **NAV** | Start command | Boustrophedon planner generates waypoints | Waypoint reached |
| **CLEAN** | Arrive at waypoint | Drive forward, vacuum + brush active | Stripe complete |
| **AVOID** | Ultrasonic < 15 cm or AI obstacle | Potential field reroute or full stop (living) | Clear path found |
| **INSPECT** | Coverage > 95% | Check for missed zones, A* to reach them | All zones done |
| **DOCK** | Battery < 15% or clean complete | A* path to dock, reduce speed | Docked |

**Inputs**: audio debris class, obstacle class, EKF position  
**Outputs**: vacuum %, brush %, motor commands

---

### 6. Dirt Heatmap

- **Storage**: NumPy grid array overlaid on the occupancy map
- **Input**: Audio debris class from Nano BLE Sense (`dust / crumbs / gravel / clear`) + current EKF position `[x, y]`
- **How**: Every time the Nano classifies a non-clear debris event, the current EKF cell `(x, y)` is incremented by a weighted score — the heatmap literally builds up wherever the robot heard dirt being picked up
- **Update**: Logged continuously during each cleaning run
- **Prediction**: After 5+ runs, highlights zones expected to be dirtiest before next clean — ARIA can start those zones first and use higher suction pre-emptively
- **Visualization**: Matplotlib PNG export — very high visual impact for demos

```python
class DirtHeatmap:
    def __init__(self, grid_shape):
        self.cumulative = np.zeros(grid_shape, dtype=np.float32)
        self.run_count = 0
    
    def log_debris(self, ekf_x, ekf_y, debris_class, confidence):
        """Called every time Nano BLE Sense sends a classification result"""
        weights = {'clear': 0.0, 'dust': 0.3, 'crumbs': 0.7, 'gravel': 1.0}
        cell_x = int(ekf_x / CELL_SIZE_CM)
        cell_y = int(ekf_y / CELL_SIZE_CM)
        score = weights.get(debris_class, 0) * confidence
        self.cumulative[cell_y][cell_x] += score
    
    def predict_next_clean(self):
        """After 5+ runs, show which zones accumulate dirt fastest"""
        if self.run_count >= 5:
            return self.cumulative / self.run_count  # avg dirt score per cell
        return None
    
    def get_priority_zones(self, top_n=5):
        """Return the N dirtiest cells to clean first next run"""
        flat = self.cumulative.flatten()
        indices = np.argsort(flat)[-top_n:][::-1]
        return [(i % self.cumulative.shape[1], i // self.cumulative.shape[1])
                for i in indices]
```

---

## Data Flow

```
Sensors → STM32 C++ → App Lab Bridge → Python EKF + Nav → Edge Impulse ×2 → Cleaning Logic → Bridge ↓ → Motors + Actuators
                                                              ↑
                                       Nano BLE Sense ────────┘ (audio debris class via BLE/UART)
```

### Every 50 ms (Uplink — STM32 → Python)

```
S,u1,u2,u3,u4,u5,u6,encL,encR,gz,timestamp\n
```

| Field | Description |
|-------|-------------|
| `u1–u6` | 6 ultrasonic distances (cm) |
| `encL`, `encR` | Encoder tick deltas |
| `gz` | IMU gyroscope Z (yaw rate) |
| `timestamp` | Millisecond timestamp for sync |

### Every 20 ms (Downlink — Python → STM32)

```
M,leftPWM,rightPWM\n     (motor command)
V,vacuumPWM\n             (vacuum speed)
B,brushPWM\n              (brush speed)
```

---

## How to Use

1. **Assemble hardware** — connect motors, sensors, and actuators per the wiring diagram.
2. **Flash the STM32 sketch** — upload the Arduino C++ code via App Lab.
3. **Deploy the Python app** — push the Linux-side Python scripts.
4. **Run the app** in Arduino App Lab.
5. **Open the web interface** at `<UNO-Q-IP-ADDRESS>:7000`.
6. **Monitor ARIA** — view live sensor data, room map, and cleaning progress.

---

## Phase 3 — EKF & Occupancy Grid Setup

### 1. Install Dependencies

```bash
# On the Arduino UNO Q Linux terminal (or your dev machine for testing):
pip install -r python/requirements.txt
```

Phase 3 requires only two packages:

| Package | Purpose | Version |
|---------|---------|--------|
| `filterpy` | Extended Kalman Filter | ≥ 1.4.5 |
| `numpy` | Occupancy grid (vectorised) | ≥ 1.24.0 |

### 2. Configure Your Hardware

Edit **`python/aria/config.py`** before running — these values must match your physical robot:

```python
WHEEL_BASE_CM     = 25.0   # ← measure your chassis (left wheel to right wheel)
WHEEL_DIAMETER_CM = 6.5    # ← measure your wheel
TICKS_PER_REV     = 360    # ← check your encoder datasheet
CELL_SIZE_CM      = 30     # ← 30 cm grid cells (works for most rooms)
```

### 3. Run in Simulator Mode (no hardware required)

```bash
# From the python/ directory:
python aria_main.py
```

This launches the Bridge **simulator** — a virtual 4 m × 4 m room with a centre obstacle.
The robot follows a boustrophedon lawnmower pattern and you'll see the ASCII map update live:

```
[ARIA Grid] coverage=12.4%  cleaned=7  known=56
· · · · · · █ █ █ · · · ·
· · · · · · █ █ █ · · · ·
· · · ✓ ✓ ✓ █ █ █ · · · ·
· · · R ✓ ✓ ░ ░ ░ · · · ·
█ █ █ █ █ █ █ █ █ █ █ █ █
```

`R` = robot  `✓` = cleaned  `░` = free  `█` = wall  `·` = unknown

### 4. Run with Real Hardware

```bash
python aria_main.py --port /dev/ttyUSB0 --baud 115200
```

> **Note**: `bridge_hw.py` (Phase 2) must be implemented first for real hardware.
> The `--port` flag will warn and fall back to the simulator if the hardware bridge isn't ready.

### 5. Load a Previous Grid (persistent maps)

```bash
# Save is automatic every 60 s to /tmp/aria_grid.npy
# Resume from a saved grid:
python aria_main.py --load /tmp/aria_grid.npy
```

### 6. Module Reference

#### `ARIALocalization` (ekf.py)

```python
from aria import ARIALocalization

ekf = ARIALocalization(start_x=0.0, start_y=0.0, start_theta=0.0)

ekf.predict(enc_left_delta, enc_right_delta)  # call every 50 ms
ekf.correct_imu(gyro_z_rad_s, dt=0.05)        # call after predict
ekf.wall_snap('left', wall_x_cm)              # call when US_left < 8 cm

x, y, theta = ekf.pose   # current best estimate
```

#### `OccupancyGrid` (occupancy_grid.py)

```python
from aria import OccupancyGrid

grid = OccupancyGrid()
grid.mark_cleaned(x_cm, y_cm)
grid.update_from_ultrasonics(x, y, theta, distances_dict)
print(grid.coverage_percent())          # 0.0 – 100.0

# Find nearest uncleaned zone (for INSPECT state)
goal = grid.nearest_uncleaned(x, y)    # returns (x_cm, y_cm) or None

# Passable array for A* (Phase 5)
nav = grid.nav_array                    # np.ndarray bool

grid.save('/tmp/my_grid.npy')
grid = OccupancyGrid.load('/tmp/my_grid.npy')
```

---

## Understanding the Code

### Project Structure

```
Roomba/
├── TODO_PLAN.md                          # Prioritised 11-phase build checklist
├── app.yaml                              # App Lab project config
├── aria_full_system_architecture.html    # Interactive architecture diagram
├── assets/
│   ├── index.html                        # Web UI frontend
│   ├── app.js                            # Browser-side logic (Socket.IO)
│   ├── style.css                         # Arduino-themed CSS
│   ├── fonts/
│   ├── img/
│   ├── libs/
│   └── docs_assets/
├── python/
│   ├── requirements.txt                  # pip dependencies (per phase)
│   ├── aria_main.py                      # ★ Phase 3 entry point — run this
│   ├── aria/                             # ARIA navigation module
│   │   ├── __init__.py
│   │   ├── config.py                     # ★ Edit hardware constants here
│   │   ├── ekf.py                        # EKF localization (filterpy)
│   │   ├── occupancy_grid.py             # NumPy occupancy grid
│   │   └── bridge.py                     # Bridge simulator (no hardware needed)
│   └── main.py                           # Original object-detection app
└── README.md
```

### Python Entry Point (`python/main.py`)

```python
from arduino.app_utils import *
from arduino.app_bricks.web_ui import WebUI
from arduino.app_bricks.object_detection import ObjectDetection
from PIL import Image
import io, base64, time

object_detection = ObjectDetection()

def on_detect_objects(client_id, data):
    """Handles object detection requests from the web UI."""
    # 1. Decode base64 image from browser
    # 2. Run inference with configurable confidence threshold
    # 3. Draw bounding boxes on detected objects
    # 4. Send annotated image + detection count + timing back to browser

ui = WebUI()
ui.on_message('detect_objects', on_detect_objects)
App.run()
```

### STM32 Side (Arduino C++ — to be developed)

The real-time sketch will handle:

- **Motor PWM** — TB6612FNG driver, receiving `M,leftPWM,rightPWM` from Bridge
- **Encoder ISR** — `attachInterrupt()` ×4 for quadrature counting
- **Ultrasonic polling** — async trigger/echo cycle across 6 sensors every 50 ms
- **IMU I²C** — MPU6050 gyro Z + accel X/Y at 100 Hz with complementary filter
- **Serial protocol** — ASCII CSV uplink/downlink via App Lab Bridge

### Bridge Communication

```python
# Python side (automatic via App Lab Bridge)
bridge.get('encL')       # Read encoder left ticks
bridge.get('us_front')   # Read front ultrasonic distance
bridge.set('M', '180,200')  # Set motor PWMs
```

```cpp
// C++ side (STM32)
Serial.println("S,24,30,15,80,90,200,45,47,1.2,12345");  // Sensor packet
if (Serial.available()) { parseCommand(buf); }             // Non-blocking
```

---

## Power Budget

> **Target runtime: 3–4 hours** (without camera: saves ~500 mA)

| Component | Power Draw | % of Battery |
|-----------|-----------|-------------|
| Drive motors ×2 | 10–15 W | 30–40% |
| Vacuum motor | 8–12 W | 20–30% |
| Ultrasonic sensors ×6 | 0.5 W | 1–2% |
| IMU (MPU6050) | 0.05 W | < 1% |
| Encoders | 0 W (passive) | 0% |
| Arduino UNO Q compute | 2–3 W | 5–8% |
| **Nano 33 BLE Sense** | **0.05 W** | **< 1%** |
| USB Camera (optional) | 3–5 W | 7–15% ⚠️ |

> **Note**: Camera use is optional and significantly reduces battery life. The primary navigation relies on ultrasonic + encoder + IMU, which are extremely power-efficient.

---

## Development Timeline

| Week | Milestone |
|------|-----------|
| 1 | Hardware assembly, motor PID calibration |
| 2 | UART Bridge between STM32 ↔ Linux working |
| 3 | EKF localization + occupancy grid running |
| 4 | Boustrophedon coverage + A* dock return |
| 5 | Nano BLE Sense audio model trained + deployed |
| 6 | Dirt heatmap from audio + EKF position |
| 7 | Web UI dashboard + manual control + live camera |
| 8 | Telegram bot — commands + automatic alerts |
| 9 | FOMO obstacle classifier (camera) deployed |
| 10+ | Competition polish + demo video |

---

## Key Advantages of the UNO Q Platform

| Capability | Traditional UNO | UNO Q |
|-----------|----------------|-------|
| Run SLAM / EKF | ❌ No RAM | ✅ 2–4 GB RAM |
| Store room maps | ❌ 2 KB | ✅ 16–32 GB storage |
| Process USB camera | ❌ Not possible | ✅ Native UVC support |
| Run TensorFlow / EI models | ❌ Too large | ✅ GPU accelerated (Adreno 702) |
| Python development | ❌ Need bridge | ✅ Full Python 3 |
| BLE / WiFi | ❌ External module | ✅ WiFi 5 + Bluetooth 5.1 built-in |
| Development speed | ⚠️ Slow | ✅ Fast with App Lab |

---

## Telegram Bot Integration

ARIA includes a **Telegram bot** (based on the `arduino:telegram_bot` Brick) giving you full remote control and real-time alerts from anywhere with a phone.

### Commands

| Command | What It Does |
|---------|-------------|
| `/start` | Welcome message + current robot status |
| `/status` | Battery %, coverage %, current state (NAV/CLEAN/DOCK...), EKF position |
| `/clean` | Start a full autonomous cleaning run |
| `/stop` | Immediately stop the robot |
| `/dock` | Send robot to docking station now (A* path) |
| `/photo` | Capture a JPEG snapshot and send it to the chat |
| `/video [sec]` | Record N seconds of video + audio, send the clip |
| `/map` | Send the current occupancy grid as a PNG |
| `/heatmap` | Send the current dirt heatmap overlay as a PNG |
| `/summary` | End-of-clean report: duration, coverage %, dirt zones found, battery used |
| `/alerts on\|off` | Enable or disable automatic push notifications |
| `/help` | List all available commands |

### Automatic Push Alerts

ARIA sends these **without you asking** (requires `/alerts on`):

| Event | Alert Content |
|-------|---------------|
| Pet or person detected | 🐾 "Obstacle detected — [pet/person]" + photo + map position |
| Path blocked > 30s | ⚠️ "Robot stuck at (x, y) cm" + photo |
| Battery < 20% | 🔋 "Low battery — returning to dock" |
| Cleaning complete | ✅ Summary message + map PNG + heatmap PNG |
| High dirt zone found | 🧹 "Heavy debris detected at zone (x, y)" |
| Error / crash | 🚨 Error message + last known position |

### Implementation Sketch

```python
from telegram.ext import Application, CommandHandler
import asyncio

ALLOWED_USERS = [123456789]  # Your Telegram user ID — keep this private!

def auth(func):
    """Decorator: reject commands from unknown users"""
    async def wrapper(update, context):
        if update.effective_user.id not in ALLOWED_USERS:
            await update.message.reply_text("Unauthorized.")
            return
        return await func(update, context)
    return wrapper

@auth
async def cmd_photo(update, context):
    """Capture snapshot and send to Telegram"""
    frame = camera.read()
    cv2.imwrite('/tmp/snap.jpg', frame)
    await context.bot.send_photo(
        chat_id=update.effective_chat.id,
        photo=open('/tmp/snap.jpg', 'rb'),
        caption=f"📷 ARIA @ position ({ekf.x[0]:.0f}, {ekf.x[1]:.0f}) cm"
    )

@auth
async def cmd_status(update, context):
    msg = (
        f"🤖 *ARIA Status*\n"
        f"State: `{cleaning_state}`\n"
        f"Position: `({ekf.x[0]:.0f}, {ekf.x[1]:.0f}) cm`\n"
        f"Coverage: `{grid.coverage_percent():.1f}%`\n"
        f"Battery: `{battery_percent:.0f}%`\n"
        f"Debris: `{last_debris_class}` ({last_debris_conf:.0%})"
    )
    await update.message.reply_text(msg, parse_mode='Markdown')

async def push_alert(bot, chat_id, message, photo_path=None):
    """Send automatic alert — called from cleaning logic"""
    await bot.send_message(chat_id=chat_id, text=message)
    if photo_path:
        await bot.send_photo(chat_id=chat_id, photo=open(photo_path, 'rb'))
```

> **Security**: Always whitelist allowed Telegram user IDs. Never deploy with an open bot.

---

## Manual Control & Live Camera

The ARIA web UI (at `<UNO-Q-IP>:7000`) includes a **full manual control panel** alongside the autonomous dashboard. Switch between AUTO and MANUAL mode at any time.

### Controls

| Control | Function |
|---------|----------|
| **D-pad / WASD** | Drive the robot (hold to move, release to stop) |
| **Speed slider** | Set maximum drive speed (10–100%) |
| **Vacuum toggle** | On/Off button for vacuum motor |
| **Brush speed slider** | 0–100% brush roller speed |
| **Take Snapshot** | Capture JPEG + timestamp + EKF position → saved + shown in UI |
| **Start Recording** | Begin recording video + audio via UNO Q camera + mic |
| **Stop Recording** | End recording — file saved to `/home/arduino/recordings/` |
| **Download Recording** | Direct download link appears after recording stops |
| **Stream toggle** | Show/hide live MJPEG camera stream in the dashboard |

### Live Camera Stream

The camera streams as **MJPEG over WebSocket** from OpenCV on the Linux side:

```python
import cv2
from flask import Response

def generate_stream():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    while True:
        ret, frame = cap.read()
        if not ret: break
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
               + buffer.tobytes() + b'\r\n')

@app.route('/stream')
def video_feed():
    return Response(generate_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
```

### Video + Audio Recording

```python
import subprocess

def start_recording(filename):
    """Record video + audio using ffmpeg on the UNO Q Linux side"""
    cmd = [
        'ffmpeg', '-y',
        '-f', 'v4l2', '-i', '/dev/video0',       # Camera
        '-f', 'alsa', '-i', 'hw:0',               # Microphone
        '-c:v', 'libx264', '-preset', 'ultrafast',
        '-c:a', 'aac', '-shortest',
        f'/home/arduino/recordings/{filename}.mp4'
    ]
    return subprocess.Popen(cmd)

def stop_recording(process):
    process.terminate()
    process.wait()
```

### D-Pad Control (JavaScript → Python)

```javascript
// Browser side — sends direction on keydown/keyup
const keyMap = { w: [200,200], s: [-180,-180], a: [0,180], d: [180,0] };

document.addEventListener('keydown', e => {
    const pwm = keyMap[e.key];
    if (pwm) socket.emit('manual_drive', { left: pwm[0], right: pwm[1] });
});
document.addEventListener('keyup', () => {
    socket.emit('manual_drive', { left: 0, right: 0 });  // Stop
});
```

```python
# Python side — forwards to STM32 via Bridge
@ui.on_message('manual_drive')
def handle_manual(client_id, data):
    left  = clamp(data['left'],  -255, 255)
    right = clamp(data['right'], -255, 255)
    bridge.set('M', f'{left},{right}')
```

---

## Build Plan

See [`TODO_PLAN.md`](TODO_PLAN.md) for the full prioritized 11-phase implementation checklist, from core hardware to competition polish.

---

## License

SPDX-License-Identifier: MPL-2.0  
Copyright © Arduino s.r.l. and/or its affiliated companies