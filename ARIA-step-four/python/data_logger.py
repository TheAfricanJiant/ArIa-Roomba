import time
import math
import io
import csv
import logging

log = logging.getLogger(__name__)

def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi

class EdgeImpulseLogger:
    def __init__(self):
        self.recording = False
        self.waypoints = []
        self.current_goal = None
        
        # We store data as a list of segments. Each segment is a list of rows (dict or tuple)
        # corresponding to the data recorded for one waypoint.
        self.segments = []
        self.current_segment = []
        
        self.arrival_cm = 10.0
        self.last_step_time = 0.0
        self.sample_interval = 0.05  # 20 Hz

        # Manual PWM inputs injected by main.py
        self.manual_l = 0
        self.manual_r = 0
        
        self.on_waypoint_reached_cb = None

    def start_collection(self, waypoints):
        """Start a new data collection session with a list of waypoints."""
        self.waypoints = [{"x": float(p["x"]), "y": float(p["y"])} for p in waypoints]
        self.segments = []
        self.current_segment = []
        self.recording = True
        self._pop_next_goal()
        log.info(f"EdgeImpulseLogger: Started recording for {len(self.waypoints) + (1 if self.current_goal else 0)} waypoints.")

    def _pop_next_goal(self):
        if self.waypoints:
            self.current_goal = self.waypoints.pop(0)
            self.current_segment = []  # Start a fresh segment
        else:
            self.current_goal = None
            self.recording = False  # Auto-stop when all waypoints are done

    def delete_last_waypoint(self):
        """Discard the current or most recently finished segment."""
        # If currently recording a segment, clear it.
        # If already paused/reached, pop the last finished segment.
        if self.current_segment:
            self.current_segment = []
            log.info("EdgeImpulseLogger: Cleared current in-progress segment.")
        elif self.segments:
            self.segments.pop()
            log.info("EdgeImpulseLogger: Cleared last completed segment.")

    def stop_collection(self):
        self.recording = False
        self.current_goal = None

    def get_csv_buffer(self):
        """Returns a string buffer containing all recorded segments in CSV format."""
        out = io.StringIO()
        writer = csv.writer(out)
        
        # Header
        writer.writerow([
            "timestamp", "dist_to_goal", "heading_error", "lin_vel", "ang_vel", 
            "us_front", "us_left", "us_right", "left_pwm", "right_pwm"
        ])
        
        # Write all finished segments
        for seg in self.segments:
            for row in seg:
                writer.writerow(row)
                
        # Write current segment (if currently recording and not yet finished)
        for row in self.current_segment:
            writer.writerow(row)
            
        return out.getvalue()

    def get_status(self):
        return {
            "recording": self.recording,
            "current_goal": self.current_goal,
            "waypoints_left": len(self.waypoints),
            "segments_recorded": len(self.segments),
            "current_segment_rows": len(self.current_segment)
        }

    def set_manual_pwm(self, left, right):
        """Called by main.py whenever manual drive changes."""
        self.manual_l = left
        self.manual_r = right

    def step(self, pose: dict, telemetry: dict, ultrasonics: dict):
        """Called inside the main navigation loop at 50Hz."""
        if not self.recording or not self.current_goal:
            return

        now = time.time()
        if now - self.last_step_time < self.sample_interval:
            return  # Throttle to 20Hz
            
        self.last_step_time = now

        # 1. Calculate distance and heading to goal
        dx = self.current_goal["x"] - pose["x_cm"]
        dy = self.current_goal["y"] - pose["y_cm"]
        dist = math.hypot(dx, dy)

        # 2. Check arrival
        if dist < self.arrival_cm:
            log.info(f"EdgeImpulseLogger: Waypoint reached. ({self.current_goal})")
            if self.current_segment:
                self.segments.append(self.current_segment)
            self.current_segment = []
            
            self._pop_next_goal()
            
            if self.on_waypoint_reached_cb:
                self.on_waypoint_reached_cb()
                
            return # Skip recording this tick since we just switched/stopped

        # 3. Calculate heading error
        desired_hdg = math.atan2(dy, dx)
        err_rad = _wrap(desired_hdg - pose["theta_rad"])

        # 4. Extract telemetry features
        # lin_vel = average of wheel rates
        lin_vel = (telemetry.get("wheel_l_cm_s", 0) + telemetry.get("wheel_r_cm_s", 0)) / 2.0
        ang_vel = telemetry.get("gyro_z", 0.0)
        
        us_f = ultrasonics.get("front", 999.0)
        us_l = ultrasonics.get("left", 999.0)
        us_r = ultrasonics.get("right", 999.0)

        # 5. Record row
        row = (
            int(now * 1000),             # timestamp (ms)
            round(dist, 2),              # dist_to_goal
            round(err_rad, 4),           # heading_error (rad)
            round(lin_vel, 2),           # lin_vel
            round(ang_vel, 4),           # ang_vel
            round(us_f, 1),              # us_front
            round(us_l, 1),              # us_left
            round(us_r, 1),              # us_right
            int(self.manual_l),          # left_pwm (LABEL)
            int(self.manual_r)           # right_pwm (LABEL)
        )
        self.current_segment.append(row)
