#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Bool
from cv_bridge import CvBridge


class CheckersPerceptionNode(Node):
    def __init__(self):
        super().__init__("checkers_perception")

        self.bridge = CvBridge()

        # -------------------------
        # Input source
        # -------------------------
        self.declare_parameter("input_mode", "ros")
        self.declare_parameter("bag_path", "")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")

        # -------------------------
        # Board crop / zoom
        # -------------------------
        self.declare_parameter("crop_x_min", 0.28)
        self.declare_parameter("crop_x_max", 0.72)
        self.declare_parameter("crop_y_min", 0.18)
        self.declare_parameter("crop_y_max", 0.82)
        self.declare_parameter("zoom_scale", 2.0)

        # -------------------------
        # ArUco marker IDs
        # -------------------------
        self.declare_parameter("top_left_id", 3)
        self.declare_parameter("top_right_id", 1)
        self.declare_parameter("bottom_right_id", 0)
        self.declare_parameter("bottom_left_id", 2)

        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("max_missing_aruco_frames", 10)
        self.declare_parameter("aruco_smoothing_alpha", 0.85)

        # -------------------------
        # Board processing
        # -------------------------
        self.declare_parameter("output_size", 800)
        self.declare_parameter("chessboard_rows", 7)
        self.declare_parameter("chessboard_cols", 7)
        self.declare_parameter("stable_required_frames", 8)

        # -------------------------
        # Piece colour thresholds HSV
        # -------------------------
        self.declare_parameter("green_lower", [45, 80, 80])
        self.declare_parameter("green_upper", [85, 255, 255])

        self.declare_parameter("purple_lower", [105, 50, 50])
        self.declare_parameter("purple_upper", [145, 255, 255])

        self.declare_parameter("piece_min_ratio", 0.18)
        self.declare_parameter("cell_crop_margin_ratio", 0.22)

        # -------------------------
        # Output topics
        # -------------------------
        self.declare_parameter("board_state_topic", "/checkers/board_state")
        self.declare_parameter("board_blocked_topic", "/checkers/board_blocked")
        self.declare_parameter("warped_view_topic", "/checkers/warped_view")
        self.declare_parameter("board_outline_topic", "/checkers/board_outline")

        # -------------------------
        # Read params
        # -------------------------
        self.input_mode = self.get_parameter("input_mode").value
        self.bag_path = self.get_parameter("bag_path").value
        self.image_topic = self.get_parameter("image_topic").value

        self.crop_x_min = float(self.get_parameter("crop_x_min").value)
        self.crop_x_max = float(self.get_parameter("crop_x_max").value)
        self.crop_y_min = float(self.get_parameter("crop_y_min").value)
        self.crop_y_max = float(self.get_parameter("crop_y_max").value)
        self.zoom_scale = float(self.get_parameter("zoom_scale").value)

        self.TOP_LEFT_ID = int(self.get_parameter("top_left_id").value)
        self.TOP_RIGHT_ID = int(self.get_parameter("top_right_id").value)
        self.BOTTOM_RIGHT_ID = int(self.get_parameter("bottom_right_id").value)
        self.BOTTOM_LEFT_ID = int(self.get_parameter("bottom_left_id").value)

        self.aruco_dictionary_name = self.get_parameter("aruco_dictionary").value
        self.max_missing_aruco_frames = int(self.get_parameter("max_missing_aruco_frames").value)
        self.src_alpha = float(self.get_parameter("aruco_smoothing_alpha").value)

        self.output_size = int(self.get_parameter("output_size").value)
        self.cell_size = self.output_size // 8

        rows = int(self.get_parameter("chessboard_rows").value)
        cols = int(self.get_parameter("chessboard_cols").value)
        self.chessboard_pattern_size = (cols, rows)

        self.stable_required_frames = int(self.get_parameter("stable_required_frames").value)

        self.green_lower = np.array(self.get_parameter("green_lower").value, dtype=np.uint8)
        self.green_upper = np.array(self.get_parameter("green_upper").value, dtype=np.uint8)

        self.purple_lower = np.array(self.get_parameter("purple_lower").value, dtype=np.uint8)
        self.purple_upper = np.array(self.get_parameter("purple_upper").value, dtype=np.uint8)

        self.piece_min_ratio = float(self.get_parameter("piece_min_ratio").value)
        self.cell_crop_margin_ratio = float(self.get_parameter("cell_crop_margin_ratio").value)

        self.board_state_topic = self.get_parameter("board_state_topic").value
        self.board_blocked_topic = self.get_parameter("board_blocked_topic").value
        self.warped_view_topic = self.get_parameter("warped_view_topic").value
        self.board_outline_topic = self.get_parameter("board_outline_topic").value

        # -------------------------
        # Publishers
        # -------------------------
        self.board_pub = self.create_publisher(Int32MultiArray, self.board_state_topic, 10)
        self.blocked_pub = self.create_publisher(Bool, self.board_blocked_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.warped_view_topic, 10)
        self.outline_pub = self.create_publisher(Image, self.board_outline_topic, 10)

        # -------------------------
        # State variables
        # -------------------------
        self.stable_count = 0
        self.last_board = None
        self.last_published_board = None

        self.last_src_pts = None
        self.last_layout_centre = None
        self.missing_aruco_count = 0

        # -------------------------
        # ArUco setup
        # -------------------------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            self.get_aruco_dictionary(self.aruco_dictionary_name)
        )
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # -------------------------
        # Input mode setup
        # -------------------------
        if self.input_mode == "ros":
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                10
            )
            self.get_logger().info(f"Using ROS image topic: {self.image_topic}")

        elif self.input_mode == "bag":
            if self.bag_path == "":
                raise RuntimeError("input_mode is 'bag' but bag_path is empty")

            import pyrealsense2 as rs

            self.rs = rs
            self.pipeline = rs.pipeline()
            self.config = rs.config()

            self.config.enable_device_from_file(self.bag_path, repeat_playback=True)
            self.config.enable_stream(rs.stream.color)

            profile = self.pipeline.start(self.config)
            playback = profile.get_device().as_playback()
            playback.set_real_time(False)

            self.timer = self.create_timer(1.0 / 30.0, self.bag_callback)
            self.get_logger().info(f"Using RealSense bag: {self.bag_path}")

        else:
            raise RuntimeError("input_mode must be either 'ros' or 'bag'")

        self.get_logger().info("Checkers perception node started")
        self.get_logger().info(
            f"Crop x=({self.crop_x_min}, {self.crop_x_max}), "
            f"y=({self.crop_y_min}, {self.crop_y_max}), zoom={self.zoom_scale}"
        )

    def get_aruco_dictionary(self, name):
        aruco_map = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        }

        if name not in aruco_map:
            self.get_logger().warn(
                f"Unknown aruco_dictionary '{name}', using DICT_4X4_50"
            )
            return cv2.aruco.DICT_4X4_50

        return aruco_map[name]

    def bag_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(1000)
            color_frame = frames.get_color_frame()

            if not color_frame:
                return

            frame = np.asanyarray(color_frame.get_data())
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            self.process_frame(frame)

        except Exception as e:
            self.get_logger().warn(f"Bag playback issue: {e}")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.process_frame(frame)

    def process_frame(self, frame):
        warped, outline_img = self.get_warped_board(frame)

        if outline_img is not None:
            self.publish_outline(outline_img)

        if warped is None:
            self.publish_blocked(True)
            return

        board_clear, corner_debug, detected_corners = self.check_chessboard_visible(warped)

        if not board_clear:
            self.publish_blocked(True)
            self.publish_debug(corner_debug)
            self.stable_count = 0
            self.last_board = None
            return

        board, debug_img = self.classify_board(warped)

        cv2.drawChessboardCorners(
            debug_img,
            self.chessboard_pattern_size,
            detected_corners,
            True
        )

        cv2.putText(
            debug_img,
            f"CHESSBOARD CLEAR | STABLE {self.stable_count}/{self.stable_required_frames}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        self.publish_debug(debug_img)

        if self.last_board is not None and np.array_equal(board, self.last_board):
            self.stable_count += 1
        else:
            self.stable_count = 1
            self.last_board = board.copy()

        if self.stable_count >= self.stable_required_frames:
            self.publish_blocked(False)

            if self.last_published_board is None or not np.array_equal(board, self.last_published_board):
                self.publish_board(board)
                self.last_published_board = board.copy()
                self.get_logger().info(f"Published board:\n{board}")

    def check_chessboard_visible(self, warped):
        debug = warped.copy()
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.equalizeHist(gray)

        found = False
        corners = None

        if hasattr(cv2, "findChessboardCornersSB"):
            flags_sb = (
                cv2.CALIB_CB_NORMALIZE_IMAGE |
                cv2.CALIB_CB_EXHAUSTIVE |
                cv2.CALIB_CB_ACCURACY
            )

            found, corners = cv2.findChessboardCornersSB(
                gray,
                self.chessboard_pattern_size,
                flags_sb
            )

        if not found:
            flags = (
                cv2.CALIB_CB_ADAPTIVE_THRESH |
                cv2.CALIB_CB_NORMALIZE_IMAGE |
                cv2.CALIB_CB_FILTER_QUADS
            )

            found, corners = cv2.findChessboardCorners(
                gray,
                self.chessboard_pattern_size,
                flags
            )

            if found:
                criteria = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001
                )

                corners = cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    criteria
                )

        expected_corners = self.chessboard_pattern_size[0] * self.chessboard_pattern_size[1]

        if found and corners is not None and len(corners) == expected_corners:
            cv2.drawChessboardCorners(
                debug,
                self.chessboard_pattern_size,
                corners,
                found
            )

            cv2.putText(
                debug,
                f"BOARD CLEAR: CHESSBOARD {expected_corners}/{expected_corners}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2
            )

            return True, debug, corners

        for r in range(1, 8):
            for c in range(1, 8):
                x = int(c * self.cell_size)
                y = int(r * self.cell_size)
                cv2.circle(debug, (x, y), 7, (0, 0, 255), -1)

        cv2.putText(
            debug,
            "BOARD BLOCKED: CHESSBOARD NOT FOUND",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2
        )

        return False, debug, None

    def get_warped_board(self, frame):
        h, w = frame.shape[:2]

        x1 = int(w * self.crop_x_min)
        x2 = int(w * self.crop_x_max)
        y1 = int(h * self.crop_y_min)
        y2 = int(h * self.crop_y_max)

        x1 = max(0, min(x1, w - 1))
        x2 = max(1, min(x2, w))
        y1 = max(0, min(y1, h - 1))
        y2 = max(1, min(y2, h))

        if x2 <= x1 or y2 <= y1:
            self.get_logger().warn("Invalid crop parameters")
            return None, frame

        roi = frame[y1:y2, x1:x2]

        roi_big = cv2.resize(
            roi,
            None,
            fx=self.zoom_scale,
            fy=self.zoom_scale,
            interpolation=cv2.INTER_LINEAR
        )

        outline_img = roi_big.copy()
        gray = cv2.cvtColor(roi_big, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        required_ids = [
            self.TOP_LEFT_ID,
            self.TOP_RIGHT_ID,
            self.BOTTOM_RIGHT_ID,
            self.BOTTOM_LEFT_ID
        ]

        detection_ok = False
        using_cache = False

        if ids is not None:
            ids_flat = ids.flatten()

            if all(tag_id in ids_flat for tag_id in required_ids):
                tag_corners = {}
                tag_centres = {}

                for corner, tag_id in zip(corners, ids_flat):
                    tag_id = int(tag_id)
                    pts = corner[0].astype(np.float32)

                    if tag_id in required_ids:
                        tag_corners[tag_id] = pts
                        tag_centres[tag_id] = np.mean(pts, axis=0)

                layout_centre_new = np.mean(
                    np.array([
                        tag_centres[self.TOP_LEFT_ID],
                        tag_centres[self.TOP_RIGHT_ID],
                        tag_centres[self.BOTTOM_RIGHT_ID],
                        tag_centres[self.BOTTOM_LEFT_ID],
                    ]),
                    axis=0
                )

                def inner_corner(tag_id):
                    pts = tag_corners[tag_id]
                    distances = np.linalg.norm(pts - layout_centre_new, axis=1)
                    return pts[np.argmin(distances)]

                src_pts_new = np.array([
                    inner_corner(self.TOP_LEFT_ID),
                    inner_corner(self.TOP_RIGHT_ID),
                    inner_corner(self.BOTTOM_RIGHT_ID),
                    inner_corner(self.BOTTOM_LEFT_ID)
                ], dtype=np.float32)

                if self.last_src_pts is None:
                    src_pts = src_pts_new.copy()
                    layout_centre = layout_centre_new.copy()
                else:
                    src_pts = (
                        self.src_alpha * self.last_src_pts +
                        (1.0 - self.src_alpha) * src_pts_new
                    )
                    layout_centre = (
                        self.src_alpha * self.last_layout_centre +
                        (1.0 - self.src_alpha) * layout_centre_new
                    )

                self.last_src_pts = src_pts.copy()
                self.last_layout_centre = layout_centre.copy()
                self.missing_aruco_count = 0
                detection_ok = True

                cv2.aruco.drawDetectedMarkers(outline_img, corners, ids)

        if not detection_ok:
            if (
                self.last_src_pts is not None and
                self.last_layout_centre is not None and
                self.missing_aruco_count < self.max_missing_aruco_frames
            ):
                src_pts = self.last_src_pts.copy()
                layout_centre = self.last_layout_centre.copy()
                self.missing_aruco_count += 1
                using_cache = True
            else:
                cv2.putText(
                    outline_img,
                    "ARUCO LOST - NO CACHE",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (0, 0, 255),
                    2
                )
                return None, outline_img

        dst_pts = np.array([
            [0, 0],
            [self.output_size - 1, 0],
            [self.output_size - 1, self.output_size - 1],
            [0, self.output_size - 1]
        ], dtype=np.float32)

        outline_pts = src_pts.astype(np.int32).reshape((-1, 1, 2))

        if using_cache:
            outline_colour = (0, 255, 255)
            text = f"USING CACHED ARUCO {self.missing_aruco_count}/{self.max_missing_aruco_frames}"
        else:
            outline_colour = (0, 255, 0)
            text = "LIVE ARUCO DETECTION"

        cv2.polylines(outline_img, [outline_pts], True, outline_colour, 4)

        for point in src_pts.astype(np.int32):
            cv2.circle(outline_img, tuple(point), 8, (0, 0, 255), -1)

        cv2.circle(
            outline_img,
            tuple(layout_centre.astype(np.int32)),
            8,
            (255, 0, 0),
            -1
        )

        cv2.putText(
            outline_img,
            text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            outline_colour,
            2
        )

        H = cv2.getPerspectiveTransform(src_pts, dst_pts)

        warped = cv2.warpPerspective(
            roi_big,
            H,
            (self.output_size, self.output_size)
        )

        return warped, outline_img

    def classify_board(self, warped):
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        board = np.zeros((8, 8), dtype=np.int32)
        debug_img = warped.copy()

        for row in range(8):
            for col in range(8):
                x1 = col * self.cell_size
                y1 = row * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size

                margin = int(self.cell_size * self.cell_crop_margin_ratio)
                crop = hsv[y1 + margin:y2 - margin, x1 + margin:x2 - margin]

                if crop.size == 0:
                    state = 0
                    label = "."
                    board[row, col] = state
                    continue

                green_mask = cv2.inRange(crop, self.green_lower, self.green_upper)
                purple_mask = cv2.inRange(crop, self.purple_lower, self.purple_upper)

                green_pixels = cv2.countNonZero(green_mask)
                purple_pixels = cv2.countNonZero(purple_mask)

                crop_area = crop.shape[0] * crop.shape[1]

                green_ratio = green_pixels / crop_area
                purple_ratio = purple_pixels / crop_area

                if green_ratio > self.piece_min_ratio and green_ratio > purple_ratio:
                    state = 1
                    label = "G"
                elif purple_ratio > self.piece_min_ratio and purple_ratio > green_ratio:
                    state = 2
                    label = "P"
                else:
                    state = 0
                    label = "."

                board[row, col] = state

                cv2.rectangle(debug_img, (x1, y1), (x2, y2), (255, 255, 255), 1)
                cv2.putText(
                    debug_img,
                    label,
                    (x1 + 35, y1 + 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (0, 0, 255),
                    3
                )

        return board, debug_img

    def publish_board(self, board):
        msg = Int32MultiArray()
        msg.data = board.flatten().tolist()
        self.board_pub.publish(msg)

    def publish_blocked(self, value):
        msg = Bool()
        msg.data = value
        self.blocked_pub.publish(msg)

    def publish_debug(self, img):
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.debug_pub.publish(msg)

    def publish_outline(self, img):
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.outline_pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, "pipeline"):
            self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CheckersPerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()