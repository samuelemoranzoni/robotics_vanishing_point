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


@dataclass
class LaneEstimate:
    """Current frame lane geometry used by debug and control."""

    valid: bool = False
    vanishing_x: float = -1.0
    vanishing_y: float = -1.0
    heading_error_norm: float = 0.0
    lateral_error_norm: float = 0.0
    confidence: float = 0.0
    left_x: float = -1.0
    right_x: float = -1.0
    lane_index: float = -1.0
    boundary_count: float = 0.0
    vp_spread_norm: float = -1.0
    lane_width_balance: float = -1.0
    x1: float = -1.0
    x2: float = -1.0
    x3: float = -1.0
    left_lane_center_x: float = -1.0
    right_lane_center_x: float = -1.0
    selected_lane_center_x: float = -1.0


class LanePerceptionNode(Node):
    """Extract simple lane geometry from the RoboMaster camera image.

    Published geometry message:
      Float32MultiArray.data =
        [valid, vanishing_x, vanishing_y, heading_error_norm,
         lateral_error_norm, confidence, left_x, right_x,
         lane_index, boundary_count, vp_spread_norm, lane_width_balance,
         x1, x2, x3, left_lane_center_x, right_lane_center_x,
         selected_lane_center_x]

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
        self.declare_parameter('hough_threshold', 24)
        self.declare_parameter('hough_min_line_length_px', 24.0)
        self.declare_parameter('hough_max_line_gap_px', 35.0)
        self.declare_parameter('max_processing_fps', 10.0)
        self.declare_parameter('draw_debug_guides', True)
        self.declare_parameter('max_boundary_count', 3)
        self.declare_parameter('temporal_filter_alpha', 0.25)

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
        self.filtered_estimate: Optional[LaneEstimate] = None

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
        boundaries = self.select_road_boundaries(lines, image_center_x, lookahead_y)
        selected, lane_index = self.select_current_lane_lines(
            boundaries, image_center_x, lookahead_y
        )
        raw_estimate = self.compute_lane_estimate(
            boundaries,
            selected,
            lane_index,
            image_center_x,
            lookahead_y,
            width,
            height,
        )
        estimate = self.filter_estimate(raw_estimate)

        self.publish_geometry(estimate)

        debug = self.draw_debug_image(
            frame,
            mask,
            lines,
            boundaries,
            selected,
            estimate,
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
        hough_threshold = int(self.get_parameter('hough_threshold').value)
        hough_min_line_length = float(
            self.get_parameter('hough_min_line_length_px').value
        )
        hough_max_line_gap = float(self.get_parameter('hough_max_line_gap_px').value)

        roi_edges = np.zeros_like(edges)
        roi_edges[roi_top:, :] = edges[roi_top:, :]

        # Hough is intentionally a bit permissive: weak/far lane boundaries can
        # be fragmented in the camera image. Later geometric checks reject
        # candidates that do not behave like road boundaries.
        raw_lines = cv2.HoughLinesP(
            roi_edges,
            rho=1,
            theta=np.pi / 180.0,
            threshold=hough_threshold,
            minLineLength=int(hough_min_line_length),
            maxLineGap=int(hough_max_line_gap),
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

    def select_road_boundaries(
        self, lines: List[LaneLine], image_center_x: float, lookahead_y: float
    ) -> List[LaneLine]:
        """Keep up to three ordered lane boundaries around the current view.

        The controller only needs the two boundaries around the robot. Keeping
        a third boundary lets the perception node validate the two-lane road
        geometry and compute a more stable vanishing point when it is visible.
        """
        if len(lines) <= 3:
            return sorted(lines, key=lambda line: line.x_at(lookahead_y))

        ordered = sorted(lines, key=lambda line: line.x_at(lookahead_y))
        intervals = list(zip(ordered, ordered[1:]))

        containing_index: Optional[int] = None
        for index, (left, right) in enumerate(intervals):
            if left.x_at(lookahead_y) <= image_center_x <= right.x_at(lookahead_y):
                containing_index = index
                break

        if containing_index is None:
            containing_index = min(
                range(len(intervals)),
                key=lambda index: abs(
                    0.5
                    * (
                        intervals[index][0].x_at(lookahead_y)
                        + intervals[index][1].x_at(lookahead_y)
                    )
                    - image_center_x
                ),
            )

        start = max(0, containing_index - 1)
        end = min(len(ordered), start + int(self.get_parameter('max_boundary_count').value))
        start = max(0, end - int(self.get_parameter('max_boundary_count').value))
        return ordered[start:end]

    def select_current_lane_lines(
        self, lines: List[LaneLine], image_center_x: float, lookahead_y: float
    ) -> Tuple[Optional[Tuple[LaneLine, LaneLine]], float]:
        """Select the two visible boundaries around the robot's current lane."""
        if len(lines) < 2:
            return None, -1.0

        ordered = sorted(lines, key=lambda line: line.x_at(lookahead_y))

        # Prefer the adjacent pair that contains the image center at the
        # lookahead row. That pair represents the lane the robot is currently in.
        for index, (left, right) in enumerate(zip(ordered, ordered[1:])):
            if left.x_at(lookahead_y) <= image_center_x <= right.x_at(lookahead_y):
                return (left, right), float(index)

        # If the robot is not exactly between two detected lines, use the pair
        # whose midpoint is closest to the image center.
        pairs = list(zip(ordered, ordered[1:]))
        best_index = min(
            range(len(pairs)),
            key=lambda index: abs(
                0.5
                * (pairs[index][0].x_at(lookahead_y) + pairs[index][1].x_at(lookahead_y))
                - image_center_x
            ),
        )
        return pairs[best_index], float(best_index)

    def compute_lane_estimate(
        self,
        boundaries: List[LaneLine],
        selected: Optional[Tuple[LaneLine, LaneLine]],
        lane_index: float,
        image_center_x: float,
        lookahead_y: float,
        width: int,
        height: int,
    ) -> LaneEstimate:
        estimate = LaneEstimate(boundary_count=float(len(boundaries)), lane_index=lane_index)
        boundary_xs = [line.x_at(lookahead_y) for line in boundaries[:3]]
        for index, value in enumerate(boundary_xs):
            if index == 0:
                estimate.x1 = value
            elif index == 1:
                estimate.x2 = value
            elif index == 2:
                estimate.x3 = value

        if len(boundary_xs) >= 2:
            estimate.left_lane_center_x = 0.5 * (boundary_xs[0] + boundary_xs[1])
        if len(boundary_xs) >= 3:
            estimate.right_lane_center_x = 0.5 * (boundary_xs[1] + boundary_xs[2])

        if selected is None:
            return estimate

        left_line, right_line = selected
        estimate.left_x = left_line.x_at(lookahead_y)
        estimate.right_x = right_line.x_at(lookahead_y)
        estimate.selected_lane_center_x = 0.5 * (estimate.left_x + estimate.right_x)
        estimate.lateral_error_norm = (
            estimate.selected_lane_center_x - image_center_x
        ) / width

        vp, vp_spread = self.compute_robust_vanishing_point(boundaries)
        if vp is not None:
            estimate.vanishing_x, estimate.vanishing_y = vp
            estimate.heading_error_norm = (estimate.vanishing_x - image_center_x) / width
        estimate.vp_spread_norm = vp_spread / width if vp_spread >= 0.0 else -1.0

        lane_width_px = abs(estimate.right_x - estimate.left_x)
        reasonable_width = 0.15 * width <= lane_width_px <= 0.95 * width
        reasonable_x = -0.5 * width <= estimate.vanishing_x <= 1.5 * width
        reasonable_y = -2.0 * height <= estimate.vanishing_y <= 1.2 * height

        estimate.lane_width_balance = self.compute_lane_width_balance(boundary_xs)
        vp_consistent = estimate.vp_spread_norm < 0.22 if len(boundaries) >= 3 else True
        estimate.valid = reasonable_x and reasonable_y and reasonable_width and vp_consistent
        estimate.confidence = self.compute_confidence(
            len(boundaries),
            estimate.valid,
            estimate.vp_spread_norm,
            estimate.lane_width_balance,
        )
        return estimate

    def compute_robust_vanishing_point(
        self, lines: List[LaneLine]
    ) -> Tuple[Optional[Tuple[float, float]], float]:
        """Average valid pairwise intersections from two or three boundaries."""
        intersections: List[Tuple[float, float]] = []
        for i, first in enumerate(lines):
            for second in lines[i + 1:]:
                point = self.compute_intersection(first, second)
                if point is not None:
                    intersections.append(point)

        if not intersections:
            return None, -1.0

        xs = np.array([point[0] for point in intersections], dtype=np.float32)
        ys = np.array([point[1] for point in intersections], dtype=np.float32)
        center = (float(np.median(xs)), float(np.median(ys)))
        if len(intersections) == 1:
            return center, 0.0

        distances = [
            math.hypot(point[0] - center[0], point[1] - center[1])
            for point in intersections
        ]
        return center, float(np.median(distances))

    def compute_lane_width_balance(self, boundary_xs: List[float]) -> float:
        """Return how similar the two lane widths are when three lines exist."""
        if len(boundary_xs) < 3:
            return -1.0
        left_width = abs(boundary_xs[1] - boundary_xs[0])
        right_width = abs(boundary_xs[2] - boundary_xs[1])
        widest = max(left_width, right_width)
        if widest < 1e-3:
            return -1.0
        return min(left_width, right_width) / widest

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

    def compute_confidence(
        self,
        boundary_count: int,
        valid: bool,
        vp_spread_norm: float,
        lane_width_balance: float,
    ) -> float:
        if not valid:
            return 0.0
        line_score = min(1.0, boundary_count / 3.0)
        if vp_spread_norm < 0.0:
            vp_score = 0.65
        else:
            vp_score = max(0.0, 1.0 - vp_spread_norm / 0.22)
        width_score = lane_width_balance if lane_width_balance >= 0.0 else 0.65
        return min(1.0, 0.35 + 0.30 * line_score + 0.20 * vp_score + 0.15 * width_score)

    def filter_estimate(self, estimate: LaneEstimate) -> LaneEstimate:
        """Smooth control-relevant values while keeping invalid frames honest."""
        if not estimate.valid:
            return estimate

        alpha = float(self.get_parameter('temporal_filter_alpha').value)
        alpha = max(0.0, min(1.0, alpha))
        previous = self.filtered_estimate
        if previous is None or not previous.valid:
            self.filtered_estimate = estimate
            return estimate

        filtered = LaneEstimate(
            valid=estimate.valid,
            vanishing_x=self.blend(previous.vanishing_x, estimate.vanishing_x, alpha),
            vanishing_y=self.blend(previous.vanishing_y, estimate.vanishing_y, alpha),
            heading_error_norm=self.blend(
                previous.heading_error_norm, estimate.heading_error_norm, alpha
            ),
            lateral_error_norm=self.blend(
                previous.lateral_error_norm, estimate.lateral_error_norm, alpha
            ),
            confidence=estimate.confidence,
            left_x=self.blend(previous.left_x, estimate.left_x, alpha),
            right_x=self.blend(previous.right_x, estimate.right_x, alpha),
            lane_index=estimate.lane_index,
            boundary_count=estimate.boundary_count,
            vp_spread_norm=estimate.vp_spread_norm,
            lane_width_balance=estimate.lane_width_balance,
            x1=estimate.x1,
            x2=estimate.x2,
            x3=estimate.x3,
            left_lane_center_x=estimate.left_lane_center_x,
            right_lane_center_x=estimate.right_lane_center_x,
            selected_lane_center_x=self.blend(
                previous.selected_lane_center_x, estimate.selected_lane_center_x, alpha
            ),
        )
        self.filtered_estimate = filtered
        return filtered

    def blend(self, old: float, new: float, alpha: float) -> float:
        return alpha * new + (1.0 - alpha) * old

    def publish_geometry(self, estimate: LaneEstimate) -> None:
        msg = Float32MultiArray()
        msg.data = [
            1.0 if estimate.valid else 0.0,
            float(estimate.vanishing_x),
            float(estimate.vanishing_y),
            float(estimate.heading_error_norm),
            float(estimate.lateral_error_norm),
            float(estimate.confidence),
            float(estimate.left_x),
            float(estimate.right_x),
            float(estimate.lane_index),
            float(estimate.boundary_count),
            float(estimate.vp_spread_norm),
            float(estimate.lane_width_balance),
            float(estimate.x1),
            float(estimate.x2),
            float(estimate.x3),
            float(estimate.left_lane_center_x),
            float(estimate.right_lane_center_x),
            float(estimate.selected_lane_center_x),
        ]
        self.geometry_pub.publish(msg)

    def draw_debug_image(
        self,
        frame: np.ndarray,
        mask: np.ndarray,
        lines: List[LaneLine],
        boundaries: List[LaneLine],
        selected: Optional[Tuple[LaneLine, LaneLine]],
        estimate: LaneEstimate,
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

        boundary_colors = [(255, 180, 0), (180, 180, 255), (255, 0, 255)]
        for index, line in enumerate(boundaries[:3]):
            self.draw_model_line(debug, line, boundary_colors[index], 2)
            x = int(round(line.x_at(lookahead_y)))
            y = int(round(lookahead_y))
            cv2.circle(debug, (x, y), 5, boundary_colors[index], -1)
            cv2.putText(
                debug,
                f'x{index + 1}',
                (x + 6, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                boundary_colors[index],
                1,
                cv2.LINE_AA,
            )

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

        if estimate.valid:
            cv2.circle(
                debug,
                (int(round(estimate.vanishing_x)), int(round(estimate.vanishing_y))),
                7,
                (0, 0, 255),
                -1,
            )

        status = 'VALID' if estimate.valid else 'INVALID'
        lane_name = self.lane_name(estimate)
        lines_text = [
            f'lane: {status}  lane_detected={lane_name}  conf={estimate.confidence:.2f}',
            f'vp=({estimate.vanishing_x:.1f}, {estimate.vanishing_y:.1f})',
            f'heading_error_norm={estimate.heading_error_norm:+.3f}',
            f'lateral_error_norm={estimate.lateral_error_norm:+.3f}',
            f'hough={len(lines)} boundaries={int(estimate.boundary_count)}',
            f'vp_spread={estimate.vp_spread_norm:+.3f} width_balance={estimate.lane_width_balance:+.2f}',
            f'x=[{estimate.x1:.0f}, {estimate.x2:.0f}, {estimate.x3:.0f}]',
            f'lane_centers=[{estimate.left_lane_center_x:.0f}, {estimate.right_lane_center_x:.0f}] selected={estimate.selected_lane_center_x:.0f}',
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

    def lane_name(self, estimate: LaneEstimate) -> str:
        if not estimate.valid:
            return 'none'
        if estimate.valid and estimate.boundary_count < 3:
            return 'current'
        if estimate.lane_index == 0.0:
            return 'left'
        if estimate.lane_index == 1.0:
            return 'right'
        return 'unknown'

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
