"""
perception_node.py — ROS2 node for checkers board perception.

Processes camera images to detect the checkers board, classify pieces via HSV,
detect stacked kings via depth/contours, and publish the canonical board state.

Architecture:
    - Subscribes to /camera/color/image_raw (sensor_msgs/Image)
    - Subscribes to /camera/depth/image_rect_raw (sensor_msgs/Image, optional)
    - Uses ArUco markers at the 4 corners to warp perspective to an overhead 8x8 grid
    - Classifies each of the 32 playable squares as Empty, Red/Black, or White
    - Differentiates Men vs Kings based on height/contours
    - Publishes std_msgs/UInt8MultiArray to /checkers/board_state (flat 64 array)

Dependencies:
    - opencv-python (cv2)
    - cv_bridge
    - numpy
"""

from __future__ import annotations
import logging
import numpy as np
from typing import List, Dict, Tuple, Optional

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import UInt8MultiArray, String
    from cv_bridge import CvBridge
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from ..game_engine.board import (
    PLAYABLE_SQUARES, INTERNAL_TO_ROWCOL,
    WHITE_MAN, WHITE_KING, BLACK_MAN, BLACK_KING, FREE,
)

logger = logging.getLogger(__name__)


class BoardPerception:
    """Core computer vision pipeline for checkers board perception.

    Decoupled from ROS2 to allow standalone testing with recorded images/videos.
    """

    def __init__(
        self,
        canonical_size: int = 800,
        marker_ids: List[int] = [0, 1, 2, 3],
        hsv_ranges: Optional[Dict[str, Tuple[np.ndarray, np.ndarray]]] = None,
    ):
        """
        Args:
            canonical_size: Size of the warped overhead square board image (px).
            marker_ids: Expected ArUco marker IDs at [top-left, top-right,
                        bottom-right, bottom-left] of the board.
            hsv_ranges: Custom HSV thresholds for piece classification.
        """
        self.size = canonical_size
        self.sq_size = canonical_size // 8
        self.marker_ids = marker_ids

        # Set up ArUco detector compatible with OpenCV 4.x+
        if HAS_CV2:
            try:
                # OpenCV 4.7+ API
                self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                self.aruco_params = cv2.aruco.DetectorParameters()
                self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            except AttributeError:
                # Older OpenCV API
                self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
                self.aruco_params = cv2.aruco.DetectorParameters_create()
                self.detector = None

        # Default HSV classification ranges
        # Pieces are typically Red (Human/Black side) and White (Robot side)
        if hsv_ranges is None:
            self.hsv_ranges = {
                # Red pieces wrap around Hue 0/180
                'red_low': (np.array([0, 100, 50]), np.array([10, 255, 255])),
                'red_high': (np.array([170, 100, 50]), np.array([180, 255, 255])),
                # Black pieces (if using pure black instead of red)
                'black': (np.array([0, 0, 0]), np.array([180, 255, 50])),
                # White pieces
                'white': (np.array([0, 0, 150]), np.array([180, 50, 255])),
            }
        else:
            self.hsv_ranges = hsv_ranges

        self.last_homography: Optional[np.ndarray] = None

    def detect_markers(self, img: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Detect ArUco markers and return their corners and IDs."""
        if not HAS_CV2:
            return None, None

        if getattr(self, 'detector', None) is not None:
            corners, ids, _ = self.detector.detectMarkers(img)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)

        return corners, ids

    def compute_homography(self, img: np.ndarray) -> Optional[np.ndarray]:
        """Compute the perspective transform matrix to overhead view.

        Finds the 4 corner markers and maps them to canonical image corners.
        Maintains the last valid homography if detection fails in a frame.
        """
        corners, ids = self.detect_markers(img)
        if ids is None or len(ids) < 4:
            return self.last_homography

        # Flatten IDs
        ids = ids.flatten()

        # Map detected markers by ID
        marker_map = {}
        for i, marker_id in enumerate(ids):
            if marker_id in self.marker_ids:
                # Use the marker center
                center = corners[i][0].mean(axis=0)
                marker_map[marker_id] = center

        # Verify all 4 corners found
        if len(marker_map) < 4:
            return self.last_homography

        # Source points corresponding to expected marker IDs
        # Order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
        src_pts = np.array([
            marker_map[self.marker_ids[0]],
            marker_map[self.marker_ids[1]],
            marker_map[self.marker_ids[2]],
            marker_map[self.marker_ids[3]],
        ], dtype=np.float32)

        # Destination points in the canonical overhead image
        # Offset slightly inwards if markers are outside the 8x8 squares,
        # assuming markers sit exactly at the outer boundary corners.
        dst_pts = np.array([
            [0, 0],
            [self.size - 1, 0],
            [self.size - 1, self.size - 1],
            [0, self.size - 1],
        ], dtype=np.float32)

        H = cv2.getPerspectiveTransform(src_pts, dst_pts)
        self.last_homography = H
        return H

    def warp_board(self, img: np.ndarray, H: np.ndarray) -> np.ndarray:
        """Warp the camera image to the canonical square board view."""
        return cv2.warpPerspective(img, H, (self.size, self.size))

    def classify_square(
        self,
        roi_bgr: np.ndarray,
        roi_depth: Optional[np.ndarray] = None,
    ) -> int:
        """Classify the content of a single board square ROI.

        Returns:
            Piece integer constant: 0 (empty), 5 (white man), 9 (white king),
            6 (black man), or 10 (black king).
        """
        # Convert ROI to HSV for robust colour segmentation
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

        # Create masks for each piece type
        mask_red1 = cv2.inRange(hsv, *self.hsv_ranges['red_low'])
        mask_red2 = cv2.inRange(hsv, *self.hsv_ranges['red_high'])
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        mask_black = cv2.inRange(hsv, *self.hsv_ranges['black'])
        mask_red_black = cv2.bitwise_or(mask_red, mask_black)

        mask_white = cv2.inRange(hsv, *self.hsv_ranges['white'])

        # Count matching pixels
        total_px = roi_bgr.shape[0] * roi_bgr.shape[1]
        red_ratio = cv2.countNonZero(mask_red_black) / total_px
        white_ratio = cv2.countNonZero(mask_white) / total_px

        threshold = 0.2  # At least 20% of ROI must match piece colour

        if red_ratio > threshold and red_ratio > white_ratio:
            base_piece = BLACK_MAN  # 6
            is_king = self._detect_king(roi_bgr, roi_depth, mask_red_black)
            return BLACK_KING if is_king else base_piece

        elif white_ratio > threshold and white_ratio > red_ratio:
            base_piece = WHITE_MAN  # 5
            is_king = self._detect_king(roi_bgr, roi_depth, mask_white)
            return WHITE_KING if is_king else base_piece

        return 0  # Empty square

    def _detect_king(
        self,
        roi_bgr: np.ndarray,
        roi_depth: Optional[np.ndarray],
        piece_mask: np.ndarray,
    ) -> bool:
        """Determine if a detected piece is a King (double stacked).

        Uses depth averages if available, otherwise looks for inner circular
        contours/features indicating a stacked piece.
        """
        # Strategy A: Use Depth Map
        if roi_depth is not None:
            # Average depth inside the piece mask
            valid_depths = roi_depth[piece_mask > 0]
            if len(valid_depths) > 0:
                mean_z = np.mean(valid_depths)
                # Compare against calibrated board surface Z
                # Stacked king is typically 12mm vs single piece 6mm
                # (Threshold values depend on depth unit scaling, assuming mm)
                KING_HEIGHT_THRESHOLD_MM = 9.0
                return float(mean_z) > KING_HEIGHT_THRESHOLD_MM

        # Strategy B: Visual Contours (Fallback)
        # Look for concentric circles or a second distinct circular edge inside
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20,
            param1=50, param2=30, minRadius=5, maxRadius=self.sq_size // 2
        )

        # If multiple strong circular edges are found near the center, it's stacked
        if circles is not None and len(circles[0]) > 1:
            return True

        return False

    def process_image(
        self,
        img_bgr: np.ndarray,
        img_depth: Optional[np.ndarray] = None,
    ) -> Tuple[List[int], Optional[np.ndarray]]:
        """Process a full frame to extract the flat 64 board array.

        Returns:
            flat64: List of 64 integers representing the board state.
            debug_img: Annotated canonical board view for visualization.
        """
        flat64 = [0] * 64
        H = self.compute_homography(img_bgr)

        if H is None:
            logger.warning("Board homography not found — skipping frame")
            return flat64, img_bgr

        # Warp RGB and Depth
        warped_bgr = self.warp_board(img_bgr, H)
        warped_depth = self.warp_board(img_depth, H) if img_depth is not None else None

        debug_img = warped_bgr.copy()

        # Iterate over all 32 playable squares
        for sq in PLAYABLE_SQUARES:
            row, col = INTERNAL_TO_ROWCOL[sq]

            # Canonical image coordinates
            # row 0 is bottom, row 7 is top
            y_start = (7 - row) * self.sq_size
            y_end = y_start + self.sq_size
            x_start = col * self.sq_size
            x_end = x_start + self.sq_size

            # Crop central ROI (avoid square borders)
            margin = int(self.sq_size * 0.2)
            roi_bgr = warped_bgr[y_start+margin : y_end-margin, x_start+margin : x_end-margin]
            roi_depth = warped_depth[y_start+margin : y_end-margin, x_start+margin : x_end-margin] if warped_depth is not None else None

            # Classify
            piece = self.classify_square(roi_bgr, roi_depth)

            # Map to flat 64 array (index = row * 8 + col)
            idx = row * 8 + col
            flat64[idx] = piece

            # Annotate debug image
            if piece != 0:
                colour = (0, 255, 0) if piece in (WHITE_MAN, WHITE_KING) else (0, 0, 255)
                label = "K" if piece in (WHITE_KING, BLACK_KING) else "M"
                center = (x_start + self.sq_size // 2, y_start + self.sq_size // 2)
                cv2.circle(debug_img, center, self.sq_size // 3, colour, 2)
                cv2.putText(debug_img, label, (center[0]-5, center[1]+5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        return flat64, debug_img


class PerceptionNode:
    """ROS2 node wrapping the board perception pipeline."""

    def __init__(self):
        if not HAS_ROS2 or not HAS_CV2:
            logger.warning("ROS2 or OpenCV not available — PerceptionNode inactive")
            return

        rclpy.init()
        self.node = Node('perception_node')
        self.bridge = CvBridge()

        # Initialize perception engine
        self.perception = BoardPerception(canonical_size=800)

        # ─── Parameters ──────────────────────────────────────────────────
        self.node.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.node.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.node.declare_parameter('publish_rate', 2.0)  # Hz

        rgb_topic = self.node.get_parameter('rgb_topic').value
        depth_topic = self.node.get_parameter('depth_topic').value
        rate = self.node.get_parameter('publish_rate').value

        # ─── Subscribers ─────────────────────────────────────────────────
        self.node.create_subscription(Image, rgb_topic, self._rgb_cb, 10)
        self.node.create_subscription(Image, depth_topic, self._depth_cb, 10)

        # ─── Publishers ──────────────────────────────────────────────────
        self.board_pub = self.node.create_publisher(
            UInt8MultiArray, '/checkers/board_state', 10
        )
        self.debug_pub = self.node.create_publisher(
            Image, '/checkers/perception_debug', 10
        )

        # Image buffers
        self.last_rgb: Optional[np.ndarray] = None
        self.last_depth: Optional[np.ndarray] = None

        # Timer to run perception at desired rate
        self.node.create_timer(1.0 / rate, self._timer_cb)

        logger.info(f"PerceptionNode initialized subscribing to {rgb_topic}")

    def _rgb_cb(self, msg: Image):
        """Buffer latest RGB frame."""
        try:
            self.last_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            logger.error(f"CvBridge RGB error: {e}")

    def _depth_cb(self, msg: Image):
        """Buffer latest Depth frame."""
        try:
            # passthrough or 32FC1 depending on sensor configs
            self.last_depth = self.bridge.imgmsg_to_cv2(msg)
        except Exception as e:
            logger.error(f"CvBridge Depth error: {e}")

    def _timer_cb(self):
        """Periodic processing callback."""
        if self.last_rgb is None:
            return

        flat64, debug_img = self.perception.process_image(self.last_rgb, self.last_depth)

        # Publish board state
        msg = UInt8MultiArray()
        msg.data = flat64
        self.board_pub.publish(msg)

        # Publish debug visualization
        if debug_img is not None and self.debug_pub.get_subscription_count() > 0:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                self.debug_pub.publish(debug_msg)
            except Exception as e:
                logger.error(f"CvBridge debug publish error: {e}")

    def run(self):
        if HAS_ROS2:
            logger.info("PerceptionNode spinning...")
            rclpy.spin(self.node)

    def shutdown(self):
        if HAS_ROS2:
            self.node.destroy_node()
            rclpy.shutdown()


def main():
    logging.basicConfig(level=logging.INFO)
    node = PerceptionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
