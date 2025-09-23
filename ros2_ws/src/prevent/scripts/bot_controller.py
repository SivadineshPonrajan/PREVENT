#!/usr/bin/env python3

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
    TURN_RATE = 0.48              # rad/s: constant yaw rate while turning

    def __init__(self):
        super().__init__('bot_controller')

        # Params (just speed + loop rate exposed)
        self.declare_parameter('forward_speed', 0.5)   # m/s
        self.declare_parameter('control_rate', 10.0)    # Hz
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.control_rate = float(self.get_parameter('control_rate').value)

        # Pub/Sub
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.sub_cue  = self.create_subscription(Int32, '/now_turn', self._on_cue, 10)

        # Timers
        self.timer = self.create_timer(1.0 / self.control_rate, self._step)
        # State
        self.mode = 'straight'   # 'straight' | 'pause_before_turn' | 'turning' | 'idle'
        self.msg = Twist()
        self.scan = None

        # Turn intent / control
        self.pending_turn = None           # None | "left" | "right" | "end"
        self.pause_deadline = None         # rclpy.time.Time
        self.turn_dir = 0                  # +1 left, -1 right

        self.get_logger().info(
            f'bot_controller ready — straight @{self.forward_speed:.2f} m/s '
            f'(laser-driven turns; cues via /now_turn; corridor-centering enabled).'
        )

    # -------- ROS Callbacks --------
    def _on_scan(self, msg: LaserScan):
        self.scan = msg

    def _on_cue(self, msg: Int32):
        code = int(msg.data)
        self.get_logger().info(f'cue code received {code}.')

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
        elif code == 100:
            self.pending_turn = "uturn"
            self.get_logger().info('Cue stored: FLAME (100)')
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

    def _do_straight(self):
        # Base forward motion
        self.msg.linear.x = self.forward_speed
        self.msg.angular.z = 0.0

        # ---- Corridor centering (C++-style simple P on left/right difference) ----
        F, L, R = self.get_distance()
        if math.isfinite(L) and math.isfinite(R):
            # error = |R - L|
            err = abs(abs(R) - abs(L))

            # two-level proportional gain (like bot.cpp)
            gain = 0.05 if err > 0.5 else 0.2

            # steer toward the wider side:
            # if R > L, we are farther from right wall -> steer right (negative z)
            # if L > R, steer left (positive z)
            if R > L:
                self.msg.angular.z = -gain * err
            elif L > R:
                self.msg.angular.z = +gain * err
            else:
                self.msg.angular.z = 0.0
        else:
            # Open on one/both sides → disable centering (go straight)
            self.msg.angular.z = 0.0

        # ---- Junction handling (unchanged) ----
        if self.pending_turn is None and self.mode == 'straight':
            if math.isfinite(F) and F <= self.FRONT_TRIGGER:
                self.get_logger().info('********* ERROR: Fatal Crash of the Robot Reported *********')
                self.mode = 'idle'
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0
                self.get_logger().error("ERROR: Wall ahead and No arrows detected! STOPPED!!!")
                return

        if self.pending_turn is None:
            return

        front = F  # already computed
        if math.isfinite(front) and (front <= self.FRONT_TRIGGER or (front <= 2.5 * self.FRONT_TRIGGER and self.pending_turn == "uturn")):
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
            self.get_logger().info('The End → Safely & Successfully DOOR pass-throughed.')
            return
        
        if self.pending_turn == "uturn":
            turn_label = self.pending_turn
            self.pending_turn = None

            self.turn_dir = -1
            self.mode = 'turning'
            self.get_logger().info(
                f'Pause done → begin TURN {turn_label.upper()} '
                f'(constant yaw {self.TURN_RATE:.2f} rad/s).'
            )
            return

        if self.pending_turn in ("left", "right"):
            # IMPORTANT: clear intent at the very start of the turn
            turn_label = self.pending_turn
            self.pending_turn = None

            self.turn_dir = +1 if turn_label == "left" else -1
            self.mode = 'turning'
            self.get_logger().info(
                f'Pause done → begin TURN {turn_label.upper()} '
                f'(constant yaw {self.TURN_RATE:.2f} rad/s).'
            )
            return

        # No intent anymore → resume straight
        self.mode = 'straight'
        self.get_logger().info('Pause done → no intent, resuming straight.')

    def _do_turning(self):
        self.msg.linear.x = 0.0
        self.msg.angular.z = self.turn_dir * self.TURN_RATE

        # Read beams
        front, _, _ = self.get_distance()

        # Open if front is NaN/inf OR > 5 m
        if (not math.isfinite(front)) or (front > 5.0):
            self.msg.angular.z = 0.0
            self.mode = 'straight'
            self.get_logger().info('Turn complete (front open) → resuming straight.')
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
