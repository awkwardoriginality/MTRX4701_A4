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

        self.declare_parameter("input_mode", "bag")
        self.declare_parameter("bag_path", "")
        self.declare_parameter("image_topic", "/camera/color/image_raw")

        self.input_mode = self.get_parameter("input_mode").value
        self.bag_path = self.get_parameter("bag_path").value
        self.image_topic = self.get_parameter("image_topic").value

        self.board_pub = self.create_publisher(Int32MultiArray, "/checkers/board_state", 10)
        self.blocked_pub = self.create_publisher(Bool, "/checkers/board_blocked", 10)
        self.debug_pub = self.create_publisher(Image, "/checkers/warped_view", 10)
        self.outline_pub = self.create_publisher(Image, "/checkers/board_outline", 10)

        self.TOP_LEFT_ID = 1
        self.TOP_RIGHT_ID = 3
        self.BOTTOM_RIGHT_ID = 0
        self.BOTTOM_LEFT_ID = 2

        self.output_size = 800
        self.cell_size = self.output_size // 8

        self.stable_required_frames = 8
        self.last_board = None
        self.stable_count = 0
        self.last_published_board = None

        self.chessboard_pattern_size = (7, 7)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

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

    def draw_detected_checkerboard_corners(self, img, corners):
        if corners is None:
            return img

        out = img.copy()
        corners = corners.reshape(-1, 2)

        for i, p in enumerate(corners):
            x = int(round(p[0]))
            y = int(round(p[1]))

            cv2.circle(out, (x, y), 9, (255, 255, 0), -1)
            cv2.circle(out, (x, y), 13, (0, 0, 0), 2)

            cv2.putText(
                out,
                str(i),
                (x + 9, y - 9),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (255, 255, 0),
                2
            )

        return out

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

        # Draw detected checkerboard inner corners on final rqt display
        cv2.drawChessboardCorners(
            debug_img,
            self.chessboard_pattern_size,
            detected_corners,
            True
        )

        for i, corner in enumerate(detected_corners):
            x, y = corner.ravel().astype(int)
            cv2.circle(debug_img, (x, y), 5, (255, 0, 255), -1)
            cv2.putText(
                debug_img,
                str(i),
                (x + 5, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (255, 0, 255),
                1
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

        if found and corners is not None and len(corners) == 49:
            cv2.drawChessboardCorners(
                debug,
                self.chessboard_pattern_size,
                corners,
                found
            )

            for i, corner in enumerate(corners):
                x, y = corner.ravel().astype(int)
                cv2.circle(debug, (x, y), 5, (255, 0, 255), -1)
                cv2.putText(
                    debug,
                    str(i),
                    (x + 5, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    (255, 0, 255),
                    1
                )

            cv2.putText(
                debug,
                "BOARD CLEAR: CHESSBOARD 49/49",
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
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        outline_img = frame.copy()

        if ids is None:
            return None, outline_img

        ids = ids.flatten()

        required_ids = [
            self.TOP_LEFT_ID,
            self.TOP_RIGHT_ID,
            self.BOTTOM_RIGHT_ID,
            self.BOTTOM_LEFT_ID
        ]

        if not all(tag_id in ids for tag_id in required_ids):
            return None, outline_img

        tag_corners = {}
        tag_centres = {}

        for corner, tag_id in zip(corners, ids):
            tag_id = int(tag_id)
            pts = corner[0].astype(np.float32)

            if tag_id in required_ids:
                tag_corners[tag_id] = pts
                tag_centres[tag_id] = np.mean(pts, axis=0)

        layout_centre = np.mean(
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
            distances = np.linalg.norm(pts - layout_centre, axis=1)
            return pts[np.argmin(distances)]

        board_top_left = inner_corner(self.TOP_LEFT_ID)
        board_top_right = inner_corner(self.TOP_RIGHT_ID)
        board_bottom_right = inner_corner(self.BOTTOM_RIGHT_ID)
        board_bottom_left = inner_corner(self.BOTTOM_LEFT_ID)

        src_pts = np.array([
            board_top_left,
            board_top_right,
            board_bottom_right,
            board_bottom_left
        ], dtype=np.float32)

        dst_pts = np.array([
            [0, 0],
            [self.output_size - 1, 0],
            [self.output_size - 1, self.output_size - 1],
            [0, self.output_size - 1]
        ], dtype=np.float32)

        cv2.aruco.drawDetectedMarkers(outline_img, corners, ids.reshape(-1, 1))

        outline_pts = src_pts.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(outline_img, [outline_pts], True, (0, 255, 0), 4)

        for point in src_pts.astype(np.int32):
            cv2.circle(outline_img, tuple(point), 8, (0, 0, 255), -1)

        cv2.circle(
            outline_img,
            tuple(layout_centre.astype(np.int32)),
            8,
            (255, 0, 0),
            -1
        )

        H = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped = cv2.warpPerspective(frame, H, (self.output_size, self.output_size))

        return warped, outline_img

    def classify_board(self, warped):
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        board = np.zeros((8, 8), dtype=np.int32)
        debug_img = warped.copy()

        green_lower = np.array([35, 50, 40])
        green_upper = np.array([90, 255, 255])

        purple_lower = np.array([100, 20, 20])
        purple_upper = np.array([145, 255, 255])

        min_pixels = 500

        for row in range(8):
            for col in range(8):
                x1 = col * self.cell_size
                y1 = row * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size

                margin = int(self.cell_size * 0.22)
                crop = hsv[y1 + margin:y2 - margin, x1 + margin:x2 - margin]

                green_mask = cv2.inRange(crop, green_lower, green_upper)
                purple_mask = cv2.inRange(crop, purple_lower, purple_upper)

                green_pixels = cv2.countNonZero(green_mask)
                purple_pixels = cv2.countNonZero(purple_mask)

                if green_pixels > min_pixels and green_pixels > purple_pixels:
                    state = 1
                    label = "G"
                elif purple_pixels > min_pixels and purple_pixels > green_pixels:
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