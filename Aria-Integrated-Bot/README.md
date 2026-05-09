# ARIA Integrated Master Controller

This project combines the capabilities of the ARIA robot's EKF Navigation Dashboard, Telegram Bot control, and live Video Object Detection into a single, unified interface.

## Features

1. **Dashboard UI**:
    - **Control**: Power on/off motors, change speed, and view raw sensor data.
    - **Map**: View real-time EKF position and occupancy grid, add waypoints, and draw cleaning zones.
    - **Camera**: Watch a live stream from an attached USB camera and track real-time object detection with confidence thresholds.
2. **Telegram Bot**:
    - Control your robot anywhere using Telegram commands (`/forward`, `/stop`, `/goto`, `/status`, `/savearea`).
    - Send an image to the bot to have it run object detection.
    - Send a text message to the bot for sentiment analysis.
3. **Autonomy**:
    - Complete path-following algorithms leveraging `filterpy` Extended Kalman Filter for accurate localization.

## Hardware Setup

- **Arduino UNO Q**
- **USB-C Hub (Externally Powered)**: Required to supply power to the UNO Q, the camera, and the serial connection.
- **USB Camera**: Plugged into the Hub.
- **Robot Connection**: A USB cable running from the Hub to the Motor Controller (e.g. XRP or Nano 33 BLE).

## Running the App

1. Connect all hardware as described above.
2. Open the project in the Arduino App Lab.
3. Verify `app.yaml` includes your Telegram Token if you are using it.
4. Hit **Run**.
5. Navigate to the WebUI (typically `<board-ip>:7000`) to access the dashboard!
