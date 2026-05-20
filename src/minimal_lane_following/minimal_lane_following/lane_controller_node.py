from typing import Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class LaneControllerNode(Node):
    """Convert lane geometry errors into RoboMaster velocity commands.

    The controller is intentionally simple:

        angular_z = steering_sign * (
            kp_lateral * lateral_error_norm
            + kp_heading * heading_error_norm
        )

    The perception node can run at the camera/Hough rate. This controller runs
    on a fixed timer and always uses the most recent valid geometry estimate.
    """

    def __init__(self) -> None:
        super().__init__('lane_controller_node')

        self.declare_parameter('geometry_topic', '/lane_geometry')
        self.declare_parameter('obstacle_topic', '/obstacle_detection')
        self.declare_parameter('cmd_vel_topic', '/rm0/cmd_vel')
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('kp_lateral', 1.3)
        self.declare_parameter('kp_heading', 1.0)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('stale_timeout_sec', 0.35)
        self.declare_parameter('hold_last_valid_sec', 0.25)
        self.declare_parameter('steering_sign', -1.0)
        self.declare_parameter('enable_obstacle_avoidance', True)
        self.declare_parameter('manual_target_lane', 'current')

        geometry_topic = self.get_parameter('geometry_topic').value
        obstacle_topic = self.get_parameter('obstacle_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.geometry_sub = self.create_subscription(
            Float32MultiArray, geometry_topic, self.geometry_callback, 10
        )
        self.obstacle_sub = self.create_subscription(
            Float32MultiArray, obstacle_topic, self.obstacle_callback, 10
        )

        self.latest_geometry: Optional[Float32MultiArray] = None
        self.latest_obstacle: Optional[Float32MultiArray] = None
        self.latest_geometry_time = self.get_clock().now()
        self.last_valid_time = self.get_clock().now()
        self.last_stop_log_time = self.get_clock().now()
        self.last_cmd = Twist()

        self.timer = self.create_timer(1.0 / control_rate_hz, self.control_step)
        self.get_logger().info(
            f'Lane controller: {geometry_topic} -> {cmd_vel_topic} at {control_rate_hz:.1f} Hz'
        )

    def geometry_callback(self, msg: Float32MultiArray) -> None:
        self.latest_geometry = msg
        self.latest_geometry_time = self.get_clock().now()

    def obstacle_callback(self, msg: Float32MultiArray) -> None:
        self.latest_obstacle = msg

    def control_step(self) -> None:
        now = self.get_clock().now()
        stale_timeout = float(self.get_parameter('stale_timeout_sec').value)
        hold_last_valid = float(self.get_parameter('hold_last_valid_sec').value)

        if self.latest_geometry is None:
            self.publish_stop('waiting for lane geometry')
            return

        message_age = (now - self.latest_geometry_time).nanoseconds * 1e-9
        if message_age > stale_timeout:
            self.publish_stop('stale lane geometry')
            return

        parsed = self.parse_geometry(self.latest_geometry)
        if parsed is None:
            self.publish_stop('malformed lane geometry')
            return

        valid, heading_error, lateral_error, confidence = parsed
        if valid:
            target_lateral_error = self.select_lateral_error(self.latest_geometry, lateral_error)
            cmd = self.compute_command(heading_error, target_lateral_error, confidence)
            self.last_valid_time = now
            self.last_cmd = cmd
            self.cmd_pub.publish(cmd)
            return

        valid_age = (now - self.last_valid_time).nanoseconds * 1e-9
        if valid_age <= hold_last_valid:
            # Short perception dropouts can happen between frames. Keep moving,
            # but slow down so the robot does not charge blindly.
            cmd = Twist()
            cmd.linear.x = 0.5 * self.last_cmd.linear.x
            cmd.angular.z = 0.5 * self.last_cmd.angular.z
            self.cmd_pub.publish(cmd)
            return

        self.publish_stop('invalid lane geometry')

    def parse_geometry(
        self, msg: Float32MultiArray
    ) -> Optional[tuple[bool, float, float, float]]:
        if len(msg.data) < 6:
            return None

        valid = msg.data[0] > 0.5
        heading_error_norm = float(msg.data[3])
        lateral_error_norm = float(msg.data[4])
        confidence = float(msg.data[5])
        return valid, heading_error_norm, lateral_error_norm, confidence

    def select_lateral_error(
        self, geometry: Float32MultiArray, current_lateral_error: float
    ) -> float:
        """Return lateral error for current lane or lane-change target."""
        target_lane = self.choose_target_lane(geometry)
        if target_lane < 0.0 or len(geometry.data) < 21:
            return current_lateral_error

        left_center_x = float(geometry.data[15])
        right_center_x = float(geometry.data[16])
        image_width = float(geometry.data[19])
        image_center_x = float(geometry.data[20])
        if image_width <= 1.0:
            return current_lateral_error

        target_center_x = left_center_x if target_lane == 0.0 else right_center_x
        if target_center_x < 0.0:
            return current_lateral_error
        return (target_center_x - image_center_x) / image_width

    def choose_target_lane(self, geometry: Float32MultiArray) -> float:
        """Pick current lane, manual target, or obstacle-avoidance target."""
        if len(geometry.data) < 19:
            return -1.0

        current_lane = float(geometry.data[18])
        manual = str(self.get_parameter('manual_target_lane').value).strip().lower()
        if manual == 'left':
            return 0.0
        if manual == 'right':
            return 1.0

        obstacle_avoidance = bool(self.get_parameter('enable_obstacle_avoidance').value)
        if not obstacle_avoidance or self.latest_obstacle is None:
            return current_lane
        if len(self.latest_obstacle.data) < 4:
            return current_lane

        obstacle_valid = self.latest_obstacle.data[0] > 0.5
        obstacle_lane = float(self.latest_obstacle.data[2])
        obstacle_close = self.latest_obstacle.data[3] > 0.5
        if obstacle_valid and obstacle_close and obstacle_lane == current_lane:
            return 1.0 if current_lane == 0.0 else 0.0

        return current_lane

    def compute_command(
        self, heading_error: float, lateral_error: float, confidence: float
    ) -> Twist:
        linear_speed = float(self.get_parameter('linear_speed').value)
        kp_lateral = float(self.get_parameter('kp_lateral').value)
        kp_heading = float(self.get_parameter('kp_heading').value)
        max_angular = float(self.get_parameter('max_angular_speed').value)
        steering_sign = float(self.get_parameter('steering_sign').value)

        angular = steering_sign * (
            kp_lateral * lateral_error + kp_heading * heading_error
        )
        angular = max(-max_angular, min(max_angular, angular))

        cmd = Twist()
        cmd.linear.x = linear_speed * max(0.25, min(1.0, confidence))
        cmd.angular.z = angular
        return cmd

    def publish_stop(self, reason: str) -> None:
        self.cmd_pub.publish(Twist())
        now = self.get_clock().now()
        elapsed = (now - self.last_stop_log_time).nanoseconds * 1e-9
        if elapsed >= 1.0:
            self.get_logger().debug(f'Stopping: {reason}')
            self.last_stop_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
