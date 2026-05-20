from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


@dataclass
class LaneLine:
    """Line model in image coordinates, represented as x = m*y + b."""

    m: float
    b: float

    def x_at(self, y: float) -> float:
        return self.m * y + self.b


@dataclass
class ObstacleDetection:
    """Colored obstacle detected in the camera image."""

    detected: bool = False
    color_name: str = 'none'
    color_code: float = 0.0
    lane_index: float = -1.0
    close: bool = False
    area_norm: float = 0.0
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)

    @property
    def center_x(self) -> float:
        x, _, w, _ = self.bbox
        return x + 0.5 * w

    @property
    def bottom_y(self) -> float:
        _, y, _, h = self.bbox
        return y + h


class ObstaclePerceptionNode(Node):
    """Detect red/green obstacles and assign them to a lane.

    Published obstacle message:
      Float32MultiArray.data =
        [valid, color_code, lane_index, close, area_norm,
         center_x, bottom_y, bbox_x, bbox_y, bbox_w, bbox_h]

    color_code: 1 = red, 2 = green. lane_index: 0 = left, 1 = right.
    """

    def __init__(self) -> None:
        super().__init__('obstacle_perception_node')

        self.declare_parameter('image_topic', '/rm0/camera/image_color')
        self.declare_parameter('lane_geometry_topic', '/lane_geometry')
        self.declare_parameter('obstacle_topic', '/obstacle_detection')
        self.declare_parameter('debug_image_topic', '/obstacle_debug/image')
        self.declare_parameter('max_processing_fps', 10.0)
        self.declare_parameter('min_area_ratio', 0.003)
        self.declare_parameter('close_area_ratio', 0.025)
        self.declare_parameter('close_bottom_ratio', 0.72)

        image_topic = self.get_parameter('image_topic').value
        lane_topic = self.get_parameter('lane_geometry_topic').value
        obstacle_topic = self.get_parameter('obstacle_topic').value
        debug_topic = self.get_parameter('debug_image_topic').value

        self.bridge = CvBridge()
        self.latest_lane_geometry: Optional[Float32MultiArray] = None
        self.last_processed_time = self.get_clock().now()

        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.lane_sub = self.create_subscription(
            Float32MultiArray, lane_topic, self.lane_geometry_callback, 10
        )
        self.obstacle_pub = self.create_publisher(Float32MultiArray, obstacle_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, debug_topic, 10)

        self.get_logger().info(
            f'Obstacle perception: {image_topic} + {lane_topic} -> {obstacle_topic}, {debug_topic}'
        )

    def lane_geometry_callback(self, msg: Float32MultiArray) -> None:
        self.latest_lane_geometry = msg

    def image_callback(self, msg: Image) -> None:
        if not self.should_process_frame():
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        lane_lines = self.parse_lane_lines()
        obstacle = self.detect_colored_obstacle(frame, lane_lines, width, height)
        self.publish_obstacle(obstacle)

        debug = self.draw_debug_image(frame, obstacle)
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))

    def should_process_frame(self) -> bool:
        """Throttle obstacle processing to match the lane perception rate."""
        max_fps = float(self.get_parameter('max_processing_fps').value)
        if max_fps <= 0.0:
            return True

        now = self.get_clock().now()
        elapsed = (now - self.last_processed_time).nanoseconds * 1e-9
        if elapsed < 1.0 / max_fps:
            return False

        self.last_processed_time = now
        return True

    def parse_lane_lines(self) -> List[LaneLine]:
        """Read the three lane boundary models from /lane_geometry."""
        msg = self.latest_lane_geometry
        if (
            msg is None
            or len(msg.data) < 27
            or msg.data[0] <= 0.5
            or msg.data[9] < 3.0
        ):
            return []

        return [
            LaneLine(float(msg.data[21]), float(msg.data[22])),
            LaneLine(float(msg.data[23]), float(msg.data[24])),
            LaneLine(float(msg.data[25]), float(msg.data[26])),
        ]

    def detect_colored_obstacle(
        self, frame: np.ndarray, lane_lines: List[LaneLine], width: int, height: int
    ) -> ObstacleDetection:
        """Detect the largest red or green object in the lower camera image."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_low = cv2.inRange(hsv, (0, 90, 70), (12, 255, 255))
        red_high = cv2.inRange(hsv, (170, 90, 70), (179, 255, 255))
        red_mask = cv2.bitwise_or(red_low, red_high)
        green_mask = cv2.inRange(hsv, (40, 70, 70), (90, 255, 255))

        candidates = [
            self.extract_obstacle_candidate(red_mask, 'red', 1.0, width, height),
            self.extract_obstacle_candidate(green_mask, 'green', 2.0, width, height),
        ]
        candidates = [candidate for candidate in candidates if candidate.detected]
        if not candidates:
            return ObstacleDetection()

        obstacle = max(candidates, key=lambda candidate: candidate.area_norm)
        obstacle.lane_index = self.assign_obstacle_lane(obstacle, lane_lines)

        close_area = float(self.get_parameter('close_area_ratio').value)
        close_bottom = float(self.get_parameter('close_bottom_ratio').value)
        obstacle.close = (
            obstacle.area_norm >= close_area
            or obstacle.bottom_y >= close_bottom * height
        )
        return obstacle

    def extract_obstacle_candidate(
        self,
        mask: np.ndarray,
        color_name: str,
        color_code: float,
        width: int,
        height: int,
    ) -> ObstacleDetection:
        """Return the largest plausible blob from one HSV color mask."""
        mask[: int(0.25 * height), :] = 0

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = float(self.get_parameter('min_area_ratio').value) * width * height
        best_bbox: Optional[Tuple[int, int, int, int]] = None
        best_area = 0.0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < 6 or h < 6:
                continue
            if h / max(1, w) > 4.0:
                continue
            if area > best_area:
                best_area = area
                best_bbox = (x, y, w, h)

        if best_bbox is None:
            return ObstacleDetection()

        return ObstacleDetection(
            detected=True,
            color_name=color_name,
            color_code=color_code,
            area_norm=best_area / float(width * height),
            bbox=best_bbox,
        )

    def assign_obstacle_lane(
        self, obstacle: ObstacleDetection, lane_lines: List[LaneLine]
    ) -> float:
        """Classify obstacle lane using the lane boundaries at bbox bottom_y."""
        if not obstacle.detected or len(lane_lines) < 3:
            return -1.0

        y = obstacle.bottom_y
        xs = sorted(line.x_at(y) for line in lane_lines[:3])
        x = obstacle.center_x
        if xs[0] <= x <= xs[1]:
            return 0.0
        if xs[1] <= x <= xs[2]:
            return 1.0
        return -1.0

    def publish_obstacle(self, obstacle: ObstacleDetection) -> None:
        x, y, w, h = obstacle.bbox
        msg = Float32MultiArray()
        msg.data = [
            1.0 if obstacle.detected else 0.0,
            float(obstacle.color_code),
            float(obstacle.lane_index),
            1.0 if obstacle.close else 0.0,
            float(obstacle.area_norm),
            float(obstacle.center_x if obstacle.detected else -1.0),
            float(obstacle.bottom_y if obstacle.detected else -1.0),
            float(x),
            float(y),
            float(w),
            float(h),
        ]
        self.obstacle_pub.publish(msg)

    def draw_debug_image(
        self, frame: np.ndarray, obstacle: ObstacleDetection
    ) -> np.ndarray:
        debug = frame.copy()
        if obstacle.detected:
            x, y, w, h = obstacle.bbox
            color = (0, 0, 255) if obstacle.color_name == 'red' else (0, 255, 0)
            cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
            cv2.circle(
                debug,
                (int(round(obstacle.center_x)), int(round(obstacle.bottom_y))),
                4,
                color,
                -1,
            )

        lines = [
            f'obstacle={obstacle.color_name}',
            f'lane={self.lane_name(obstacle.lane_index)} close={int(obstacle.close)}',
            f'area={obstacle.area_norm:.3f}',
            f'center=({obstacle.center_x:.0f}, {obstacle.bottom_y:.0f})',
            f'bbox=({obstacle.bbox[0]}, {obstacle.bbox[1]}, {obstacle.bbox[2]}, {obstacle.bbox[3]})',
        ]
        y = 24
        for text in lines:
            cv2.putText(debug, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(debug, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            y += 22
        return debug

    def lane_name(self, lane_index: float) -> str:
        if lane_index == 0.0:
            return 'left'
        if lane_index == 1.0:
            return 'right'
        return 'none'


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstaclePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
