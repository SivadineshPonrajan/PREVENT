#!/usr/bin/env python3
# Minimal modular controller with laser-driven turning + simple corridor centering.
# - Vision cues are cached until wall is near (front <= 1.0 m).
# - Pause 1.0 s, then rotate at constant yaw rate.
# - Turn completes when front > 2.5 m for 3 consecutive scans (or timeout).
# - Red "end/door" stops the robot after the pause.
# - While in STRAIGHT mode, if both side beams are finite, keep robot in the middle third of corridor.

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32


class BotController(Node):
    # ---- Hardcoded behavior knobs (intentionally not ROS params) ----
    # Turning / junction logic
    FRONT_TRIGGER = 1.0          # m: start pause/turn when front <= this
    PAUSE_BEFORE_TURN = 1.0      # s: stop before turning
    TURN_RATE = 0.6              # rad/s: constant yaw rate while turning
    TURN_DONE_FRONT = 2.5        # m: declare turn done when front > this
    TURN_HYSTERESIS = 3          # scans: consecutive frames to confirm "front open"
    TURN_TIMEOUT = 7.5           # s: safety cap on turning duration

    # Corridor centering (active only in STRAIGHT mode, both sides finite)
    CENTER_DEADBAND = 0.08       # m: extra margin around [W/3, 2W/3] to avoid twitchiness
    CENTER_STRONG_ERR = 0.25     # m: beyond this, use a stronger correction
    CENTER_WZ_SMALL = 0.25       # rad/s: small steering nudge
    CENTER_WZ_MED   = 0.45       # rad/s: stronger nudge when far outside
    CENTER_OUT_HYST = 2          # scans: require this many frames outside before engaging correction
    CENTER_IN_HYST  = 2          # scans: require this many frames inside before relaxing to zero

    def __init__(self):
        super().__init__('bot_controller')

        # Params (just speed + loop rate exposed)
        self.declare_parameter('forward_speed', 0.25)   # m/s
        self.declare_parameter('control_rate', 10.0)    # Hz
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        # Pub/Sub
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.sub_cue  = self.create_subscription(Int32, '/now_turn', self._on_cue, 10)

        # Timers
        self.timer = self.create_timer(1.0 / self.control_rate, self._step)
        # self.debug_timer = self.create_timer(1.0, self._debug_ranges)

        # State
        self.mode = 'straight'   # 'straight' | 'pause_before_turn' | 'turning' | 'idle'
        self.msg = Twist()
        self.scan = None

        # Turn intent / control
        self.pending_turn = None           # None | "left" | "right" | "end"
        self.pause_deadline = None         # rclpy.time.Time
        self.turn_deadline = None          # rclpy.time.Time
        self.turn_dir = 0                  # +1 left, -1 right
        self.front_open_count = 0          # hysteresis counter

        # Centering hysteresis
        self.center_active = False         # whether we're currently applying a centering correction
        self.center_out_frames = 0
        self.center_in_frames  = 0
        self.center_last_dir = 0           # -1 steer right, +1 steer left, 0 none

        self.get_logger().info(
            f'bot_controller ready — straight @{self.forward_speed:.2f} m/s '
            f'(laser-driven turns; cues via /now_turn; corridor-centering enabled).'
        )

    # -------- ROS Callbacks --------
    def _on_scan(self, msg: LaserScan):
        self.scan = msg

    def _on_cue(self, msg: Int32):
        code = int(msg.data)
        self.get_logger().info(f'cue code received {code}).')

        # Only accept new cues when we're driving straight and no cue is latched.
        if not (self.mode == 'straight' and self.pending_turn is None):
            self.get_logger().debug(
                f'Ignoring cue {code} (mode={self.mode}, pending={self.pending_turn}).'
            )
            return

        if code == 5:
            self.pending_turn = "left"
            self.get_logger().info('Cue stored: LEFT (5).')
        elif code == 10:
            self.pending_turn = "right"
            self.get_logger().info('Cue stored: RIGHT (10).')
        elif code == 15:
            self.pending_turn = "end"
            self.get_logger().info('Cue stored: END/DOOR (15).')
        else:
            self.get_logger().debug(f'Unknown cue {code}, ignored.')

    # -------- Control Loop --------
    def _step(self):
        if self.mode == 'straight':
            self._do_straight()
        elif self.mode == 'pause_before_turn':
            self._do_pause_before_turn()
        elif self.mode == 'turning':
            self._do_turning()
        else:  # 'idle' or unknown
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0

        self.pub_cmd.publish(self.msg)

    # --- Modes ---
    def _do_straight(self):
        # Base forward motion
        self.msg.linear.x = self.forward_speed
        self.msg.angular.z = 0.0

        # ---- Corridor centering (only if both sides finite) ----
        F, L, R = self.get_distance()
        if math.isfinite(L) and math.isfinite(R):
            W = L + R
            # Target band for L: [W/3, 2W/3], expanded by deadband
            Lmin = W / 3.0 + self.CENTER_DEADBAND
            Lmax = 2.0 * W / 3.0 - self.CENTER_DEADBAND
            # self.get_logger().info(f"L: {L:.2f} m, R: {R:.2f} m, W: {W:.2f} m, Lmin: {Lmin:.2f} m, Lmax: {Lmax:.2f} m, Turn: {self.pending_turn}")

            need_correction = False
            steer_dir = 0     # -1 -> steer right, +1 -> steer left
            error_mag = 0.0

            if L < Lmin:
                need_correction = True
                steer_dir = -1
                error_mag = (Lmin - L)
            elif L > Lmax:
                need_correction = True
                steer_dir = +1
                error_mag = (L - Lmax)

            if need_correction:
                # Outside band: count up, reset in-band counter
                self.center_out_frames += 1
                self.center_in_frames = 0

                if self.center_out_frames >= self.CENTER_OUT_HYST:
                    self.center_active = True
                    self.center_last_dir = steer_dir
                    # Two-level correction based on how far outside we are
                    wz = self.CENTER_WZ_MED if error_mag >= self.CENTER_STRONG_ERR else self.CENTER_WZ_SMALL
                    self.msg.angular.z = steer_dir * wz * 0.5
            else:
                # Inside band (with deadband): count in-band frames
                self.center_in_frames += 1
                self.center_out_frames = 0
                if self.center_in_frames >= self.CENTER_IN_HYST:
                    self.center_active = False
                    self.center_last_dir = 0
                    # keep angular.z at 0

        else:
            # Open on one/both sides → disable centering (go straight)
            self.center_active = False
            self.center_out_frames = 0
            self.center_in_frames = 0
            self.center_last_dir = 0

        # ---- Junction handling (unchanged) ----
        if self.pending_turn is None and self.mode == 'straight':
            if math.isfinite(F) and F <= self.FRONT_TRIGGER:
                self.get_logger().info('********* ERROR: Fatal Crash of the Robot Reported *********')
                self.mode = 'idle'
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
                self.getlogger().error(f"ERROR: Wall ahead and No arrows detected! STOPPED!!!")
                return
        if self.pending_turn is None:
            return

        front, _, _ = self.get_distance()
        if math.isfinite(front) and front <= self.FRONT_TRIGGER:
            self.mode = 'pause_before_turn'
            self.pause_deadline = self.get_clock().now() + Duration(seconds=self.PAUSE_BEFORE_TURN)
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.get_logger().info(
                f'Front {front:.2f} m <= {self.FRONT_TRIGGER:.2f} m → pause before turn ({self.PAUSE_BEFORE_TURN:.1f}s).'
            )

    def _do_pause_before_turn(self):
        # Full stop during pause
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0

        if self.pause_deadline is None or self.get_clock().now() < self.pause_deadline:
            return

        # Pause finished — decide what to do based on stored intent
        if self.pending_turn == "end":
            self.pending_turn = None
            self.mode = 'idle'
            self.get_logger().info('Pause done → END: entering idle (stop).')
            return

        if self.pending_turn in ("left", "right"):
            # IMPORTANT: clear intent at the very start of the turn
            turn_label = self.pending_turn
            self.pending_turn = None

            self.turn_dir = +1 if turn_label == "left" else -1
            self.front_open_count = 0
            self.turn_deadline = self.get_clock().now() + Duration(seconds=self.TURN_TIMEOUT)
            self.mode = 'turning'
            self.get_logger().info(
                f'Pause done → begin TURN {turn_label.upper()} '
                f'(constant yaw {self.TURN_RATE:.2f} rad/s, timeout {self.TURN_TIMEOUT:.1f}s).'
            )
            return

        # No intent anymore → resume straight
        self.mode = 'straight'
        self.get_logger().info('Pause done → no intent, resuming straight.')

    def _do_turning(self):
        # Rotate at constant rate; no forward motion
        self.msg.linear.x = 0.0
        self.msg.angular.z = self.turn_dir * self.TURN_RATE

        # Laser-driven completion
        front, _, _ = self.get_distance()

        # "Open" if front is non-finite (inf) OR greater than threshold
        if (not math.isfinite(front)) or (front > self.TURN_DONE_FRONT):
            self.front_open_count += 1
        else:
            self.front_open_count = 0

        if self.front_open_count >= self.TURN_HYSTERESIS:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.mode = 'straight'
            self.front_open_count = 0
            # Reset centering hysteresis so we don't carry old counts across modes
            self.center_active = False
            self.center_out_frames = 0
            self.center_in_frames = 0
            self.center_last_dir = 0
            self.get_logger().info('Turn complete → front open consistently; resuming straight.')
            return

        # Safety timeout
        if self.turn_deadline is not None and self.get_clock().now() >= self.turn_deadline:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.mode = 'straight'
            self.front_open_count = 0
            self.center_active = False
            self.center_out_frames = 0
            self.center_in_frames = 0
            self.center_last_dir = 0
            self.get_logger().warn('Turn timeout reached → resuming straight.')
            return

    # -------- Helpers --------
    def _beam_at(self, scan: LaserScan, angle_rad: float) -> float:
        """Return a single ray distance at 'angle_rad' (robot frame). Falls back to neighbors if exact beam invalid."""
        if scan is None:
            return float('inf')
        a_min = scan.angle_min
        inc   = scan.angle_increment
        n     = len(scan.ranges)
        if n == 0 or inc == 0.0:
            return float('inf')

        angle = max(scan.angle_min, min(scan.angle_max, angle_rad))
        idx = int(round((angle - a_min) / inc))
        idx = max(0, min(n - 1, idx))

        candidates = [idx]
        if idx - 1 >= 0:
            candidates.append(idx - 1)
        if idx + 1 < n:
            candidates.append(idx + 1)

        for i in candidates:
            r = scan.ranges[i]
            if math.isfinite(r) and r > 0.0:
                return r
        return float('inf')

    def get_distance(self) -> tuple[float, float, float]:
        """Convenience beams: forward (0), left (+90°), right (-90°)."""
        forward = self._beam_at(self.scan, 0.0) 
        left = self._beam_at(self.scan, (math.pi/2)) 
        right = self._beam_at(self.scan, (3*math.pi/2))
        return forward, left, right

    def _debug_ranges(self):
        f, l, r = self.get_distance()
        self.get_logger().info(f"Forward: {f:.2f} m, Left: {l:.2f} m, Right: {r:.2f} m")

    # -------- (Optional) Public API --------
    def go_straight(self, speed: float = None):
        """Enter straight-driving mode at given speed."""
        if speed is not None:
            self.forward_speed = float(speed)
        self.mode = 'straight'


def main():
    rclpy.init()
    node = BotController()
    try:
        rclpy.spin(node)
    finally:
        # Stop on shutdown
        node.pub_cmd.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
