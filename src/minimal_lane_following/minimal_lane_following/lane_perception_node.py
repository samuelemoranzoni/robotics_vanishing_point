import math
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
    length: float
    p1: Tuple[int, int]
    p2: Tuple[int, int]

    def x_at(self, y: float) -> float:
        return self.m * y + self.b


class LanePerceptionNode(Node):
    """Extract simple lane geometry from the RoboMaster camera image.

    Published geometry message:
      Float32MultiArray.data =
        [valid, vanishing_x, vanishing_y, heading_error_norm,
         lateral_error_norm, confidence, left_x, right_x]

    The normalized errors are divided by image width, so they stay roughly
    independent of the camera resolution.
    """

    def __init__(self) -> None:
        super().__init__('lane_perception_node')

        self.declare_parameter('image_topic', '/rm0/camera/image_color')
        self.declare_parameter('geometry_topic', '/lane_geometry')
        self.declare_parameter('debug_image_topic', '/lane_debug/image')
        self.declare_parameter('lookahead_ratio', 0.78)
        self.declare_parameter('roi_top_ratio', 0.35)
        self.declare_parameter('min_segment_length_px', 35.0)
        self.declare_parameter('white_threshold', 185)
        self.declare_parameter('max_processing_fps', 10.0)
        self.declare_parameter('draw_debug_guides', True)

        image_topic = self.get_parameter('image_topic').value
        geometry_topic = self.get_parameter('geometry_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value

        self.bridge = CvBridge()
        self.geometry_pub = self.create_publisher(Float32MultiArray, geometry_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, debug_image_topic, 10)
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.last_processed_time = self.get_clock().now()

        self.get_logger().info(
            f'Lane perception: {image_topic} -> {geometry_topic}, {debug_image_topic}'
        )

    def image_callback(self, msg: Image) -> None:
        if not self.should_process_frame():
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        image_center_x = width / 2.0
        lookahead_y = self.get_parameter('lookahead_ratio').value * height

        mask = self.segment_white_lane_pixels(frame)
        edges = cv2.Canny(mask, 60, 160)
        lines = self.detect_lane_lines(edges, width, height)
        selected = self.select_current_lane_lines(lines, image_center_x, lookahead_y)

        valid = False
        vanishing_x = -1.0
        vanishing_y = -1.0
        heading_error_norm = 0.0
        lateral_error_norm = 0.0
        confidence = 0.0
        left_x = -1.0
        right_x = -1.0

        if selected is not None:
            left_line, right_line = selected
            left_x = left_line.x_at(lookahead_y)
            right_x = right_line.x_at(lookahead_y)
            lane_center_x = 0.5 * (left_x + right_x)
            lateral_error_norm = (lane_center_x - image_center_x) / width

            vp = self.compute_intersection(left_line, right_line)
            if vp is not None:
                vanishing_x, vanishing_y = vp
                heading_error_norm = (vanishing_x - image_center_x) / width

                # The estimate is useful only when it is not wildly outside
                # the image. This avoids steering from unstable line pairs.
                reasonable_x = -0.5 * width <= vanishing_x <= 1.5 * width
                reasonable_y = -2.0 * height <= vanishing_y <= 1.2 * height
                lane_width_px = abs(right_x - left_x)
                reasonable_width = 0.15 * width <= lane_width_px <= 0.95 * width
                valid = reasonable_x and reasonable_y and reasonable_width

            confidence = self.compute_confidence(lines, valid)

        self.publish_geometry(
            valid,
            vanishing_x,
            vanishing_y,
            heading_error_norm,
            lateral_error_norm,
            confidence,
            left_x,
            right_x,
        )

        debug = self.draw_debug_image(
            frame,
            mask,
            lines,
            selected,
            valid,
            vanishing_x,
            vanishing_y,
            heading_error_norm,
            lateral_error_norm,
            confidence,
            lookahead_y,
        )
        self.debug_image_pub.publish(
            self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        )

    def should_process_frame(self) -> bool:
        """Throttle image processing to avoid starving the simulator heartbeat."""
        max_fps = float(self.get_parameter('max_processing_fps').value)
        if max_fps <= 0.0:
            return True

        now = self.get_clock().now()
        elapsed = (now - self.last_processed_time).nanoseconds * 1e-9
        if elapsed < 1.0 / max_fps:
            return False

        self.last_processed_time = now
        return True

    def segment_white_lane_pixels(self, frame: np.ndarray) -> np.ndarray:
        """Return a binary mask for bright low-saturation lane paint."""
        white_threshold = int(self.get_parameter('white_threshold').value)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # White paint is bright and weakly saturated. The extra grayscale
        # threshold keeps the detector simple and stable in the dark-road scene.
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mask = np.where(
            (value > white_threshold) & (saturation < 95) & (gray > white_threshold),
            255,
            0,
        ).astype(np.uint8)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def detect_lane_lines(
        self, edges: np.ndarray, width: int, height: int
    ) -> List[LaneLine]:
        """Detect candidate lane lines with Hough and fit x = m*y + b."""
        roi_top = int(self.get_parameter('roi_top_ratio').value * height)
        min_length = float(self.get_parameter('min_segment_length_px').value)

        roi_edges = np.zeros_like(edges)
        roi_edges[roi_top:, :] = edges[roi_top:, :]

        raw_lines = cv2.HoughLinesP(
            roi_edges,
            rho=1,
            theta=np.pi / 180.0,
            threshold=35,
            minLineLength=int(min_length),
            maxLineGap=22,
        )
        if raw_lines is None:
            return []

        candidates: List[LaneLine] = []
        for raw in raw_lines[:, 0, :]:
            x1, y1, x2, y2 = [int(v) for v in raw]
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length < min_length or abs(dy) < 18:
                continue

            # Fit x as a function of y. This is numerically convenient because
            # lane markings are nearly vertical in the camera image.
            m = dx / dy
            b = x1 - m * y1
            bottom_x = m * (height - 1) + b
            top_x = m * roi_top + b

            # Reject line estimates that do not pass near the visible image.
            if max(bottom_x, top_x) < -0.4 * width:
                continue
            if min(bottom_x, top_x) > 1.4 * width:
                continue

            candidates.append(LaneLine(m, b, length, (x1, y1), (x2, y2)))

        return self.merge_similar_lines(candidates, height)

    def merge_similar_lines(self, lines: List[LaneLine], height: int) -> List[LaneLine]:
        """Merge nearby Hough segments into fewer, more stable line estimates."""
        if not lines:
            return []

        look_y = 0.78 * height
        lines = sorted(lines, key=lambda line: line.x_at(look_y))
        groups: List[List[LaneLine]] = []

        for line in lines:
            if not groups:
                groups.append([line])
                continue

            group_center = np.mean([item.x_at(look_y) for item in groups[-1]])
            same_position = abs(line.x_at(look_y) - group_center) < 45.0
            same_slope = abs(line.m - np.mean([item.m for item in groups[-1]])) < 0.35
            if same_position and same_slope:
                groups[-1].append(line)
            else:
                groups.append([line])

        merged: List[LaneLine] = []
        for group in groups:
            weights = np.array([line.length for line in group], dtype=np.float32)
            slopes = np.array([line.m for line in group], dtype=np.float32)
            intercepts = np.array([line.b for line in group], dtype=np.float32)
            m = float(np.average(slopes, weights=weights))
            b = float(np.average(intercepts, weights=weights))
            longest = max(group, key=lambda line: line.length)
            merged.append(LaneLine(m, b, float(np.sum(weights)), longest.p1, longest.p2))

        return merged

    def select_current_lane_lines(
        self, lines: List[LaneLine], image_center_x: float, lookahead_y: float
    ) -> Optional[Tuple[LaneLine, LaneLine]]:
        """Select the two visible boundaries around the robot's current lane."""
        if len(lines) < 2:
            return None

        ordered = sorted(lines, key=lambda line: line.x_at(lookahead_y))

        # Prefer the adjacent pair that contains the image center at the
        # lookahead row. That pair represents the lane the robot is currently in.
        for left, right in zip(ordered, ordered[1:]):
            if left.x_at(lookahead_y) <= image_center_x <= right.x_at(lookahead_y):
                return left, right

        # If the robot is not exactly between two detected lines, use the pair
        # whose midpoint is closest to the image center.
        return min(
            zip(ordered, ordered[1:]),
            key=lambda pair: abs(
                0.5 * (pair[0].x_at(lookahead_y) + pair[1].x_at(lookahead_y))
                - image_center_x
            ),
        )

    def compute_intersection(
        self, first: LaneLine, second: LaneLine
    ) -> Optional[Tuple[float, float]]:
        """Compute the intersection of two x = m*y + b lines."""
        denominator = first.m - second.m
        if abs(denominator) < 1e-3:
            return None
        y = (second.b - first.b) / denominator
        x = first.x_at(y)
        return x, y

    def compute_confidence(self, lines: List[LaneLine], valid: bool) -> float:
        if not valid:
            return 0.0
        # A tiny confidence heuristic: two good lines are enough, extra line
        # support increases confidence but is capped.
        return min(1.0, 0.45 + 0.15 * len(lines))

    def publish_geometry(
        self,
        valid: bool,
        vanishing_x: float,
        vanishing_y: float,
        heading_error_norm: float,
        lateral_error_norm: float,
        confidence: float,
        left_x: float,
        right_x: float,
    ) -> None:
        msg = Float32MultiArray()
        msg.data = [
            1.0 if valid else 0.0,
            float(vanishing_x),
            float(vanishing_y),
            float(heading_error_norm),
            float(lateral_error_norm),
            float(confidence),
            float(left_x),
            float(right_x),
        ]
        self.geometry_pub.publish(msg)

    def draw_debug_image(
        self,
        frame: np.ndarray,
        mask: np.ndarray,
        lines: List[LaneLine],
        selected: Optional[Tuple[LaneLine, LaneLine]],
        valid: bool,
        vanishing_x: float,
        vanishing_y: float,
        heading_error_norm: float,
        lateral_error_norm: float,
        confidence: float,
        lookahead_y: float,
    ) -> np.ndarray:
        debug = frame.copy()
        height, width = debug.shape[:2]

        # Add a faint mask overlay so it is obvious which pixels are being used.
        overlay = debug.copy()
        overlay[mask > 0] = (255, 255, 255)
        debug = cv2.addWeighted(debug, 0.75, overlay, 0.25, 0)

        for line in lines:
            self.draw_model_line(debug, line, (120, 120, 120), 1)

        if selected is not None:
            self.draw_model_line(debug, selected[0], (0, 255, 255), 3)
            self.draw_model_line(debug, selected[1], (0, 255, 0), 3)

        if bool(self.get_parameter('draw_debug_guides').value):
            # Blue guide lines are not detected lanes:
            # vertical = image center, horizontal = controller lookahead row.
            cv2.line(
                debug,
                (width // 2, 0),
                (width // 2, height - 1),
                (255, 0, 0),
                1,
            )
            cv2.line(
                debug,
                (0, int(lookahead_y)),
                (width - 1, int(lookahead_y)),
                (255, 0, 0),
                1,
            )

        if valid:
            cv2.circle(
                debug,
                (int(round(vanishing_x)), int(round(vanishing_y))),
                7,
                (0, 0, 255),
                -1,
            )

        status = 'VALID' if valid else 'INVALID'
        lines_text = [
            f'lane: {status}  confidence={confidence:.2f}',
            f'vp=({vanishing_x:.1f}, {vanishing_y:.1f})',
            f'heading_error_norm={heading_error_norm:+.3f}',
            f'lateral_error_norm={lateral_error_norm:+.3f}',
            f'hough_lines={len(lines)}',
        ]
        y = 24
        for text in lines_text:
            cv2.putText(
                debug,
                text,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                debug,
                text,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            y += 22

        return debug

    def draw_model_line(
        self, image: np.ndarray, line: LaneLine, color: Tuple[int, int, int], thickness: int
    ) -> None:
        height, width = image.shape[:2]
        y1 = int(0.35 * height)
        y2 = height - 1
        x1 = int(round(line.x_at(y1)))
        x2 = int(round(line.x_at(y2)))
        cv2.line(image, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LanePerceptionNode()
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
