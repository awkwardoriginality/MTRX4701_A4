#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class ChessboardMarkerNode(Node):
    def __init__(self):
        super().__init__("chessboard_marker_node")

        self.declare_parameter("frame_id", "base_link")

        self.declare_parameter("origin_x", 0.30)
        self.declare_parameter("origin_y", -0.20)
        self.declare_parameter("origin_z", 0.00)

        self.declare_parameter("square_size", 0.05)
        self.declare_parameter("line_width", 0.003)

        # 0 = 0 deg, 1 = 90 deg, 2 = 180 deg, 3 = 270 deg
        self.declare_parameter("rotation_steps", 0)

        self.pub = self.create_publisher(MarkerArray, "/chessboard_markers", 10)
        self.timer = self.create_timer(0.5, self.publish_board)

        self.get_logger().info("Chessboard marker node started")

    def rotate_square(self, row, col, rotation):
        rotation = rotation % 4

        if rotation == 0:
            return row, col
        if rotation == 1:
            return col, 7 - row
        if rotation == 2:
            return 7 - row, 7 - col
        if rotation == 3:
            return 7 - col, row

        return row, col

    def make_square(self, marker_id, x, y, z, size, is_dark):
        marker = Marker()
        marker.header.frame_id = self.get_parameter("frame_id").value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "chessboard_squares"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = x + size / 2.0
        marker.pose.position.y = y + size / 2.0
        marker.pose.position.z = z - 0.001
        marker.pose.orientation.w = 1.0

        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = 0.002

        if is_dark:
            marker.color.r = 0.1
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.65
        else:
            marker.color.r = 0.9
            marker.color.g = 0.9
            marker.color.b = 0.9
            marker.color.a = 0.65

        return marker

    def make_grid_line(self, marker_id, p1, p2):
        marker = Marker()
        marker.header.frame_id = self.get_parameter("frame_id").value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "chessboard_grid"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = self.get_parameter("line_width").value

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = [p1, p2]
        return marker

    def make_piece(self, marker_id, x, y, z, radius, height, colour):
        marker = Marker()
        marker.header.frame_id = self.get_parameter("frame_id").value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "checker_pieces"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.x = 2.0 * radius
        marker.scale.y = 2.0 * radius
        marker.scale.z = height

        if colour == "green":
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 0.2
            marker.color.a = 0.9
        else:
            marker.color.r = 0.55
            marker.color.g = 0.0
            marker.color.b = 0.8
            marker.color.a = 0.9

        return marker

    def publish_board(self):
        ox = float(self.get_parameter("origin_x").value)
        oy = float(self.get_parameter("origin_y").value)
        oz = float(self.get_parameter("origin_z").value)
        square_size = float(self.get_parameter("square_size").value)
        rotation = int(self.get_parameter("rotation_steps").value) % 4

        msg = MarkerArray()
        marker_id = 0

        # 8x8 squares
        for row in range(8):
            for col in range(8):
                r_rot, c_rot = self.rotate_square(row, col, rotation)

                x = ox + c_rot * square_size
                y = oy + r_rot * square_size

                is_dark = (row + col) % 2 == 0

                msg.markers.append(
                    self.make_square(marker_id, x, y, oz, square_size, is_dark)
                )
                marker_id += 1

        # Grid lines stay aligned to the rotated board bounding box
        for i in range(9):
            p1 = Point()
            p1.x = ox + i * square_size
            p1.y = oy
            p1.z = oz + 0.003

            p2 = Point()
            p2.x = ox + i * square_size
            p2.y = oy + 8 * square_size
            p2.z = oz + 0.003

            msg.markers.append(self.make_grid_line(marker_id, p1, p2))
            marker_id += 1

            p3 = Point()
            p3.x = ox
            p3.y = oy + i * square_size
            p3.z = oz + 0.003

            p4 = Point()
            p4.x = ox + 8 * square_size
            p4.y = oy + i * square_size
            p4.z = oz + 0.003

            msg.markers.append(self.make_grid_line(marker_id, p3, p4))
            marker_id += 1

        # Checker pieces
        piece_radius = 0.020
        piece_height = 0.020

        green_rows = [0, 1, 2]
        purple_rows = [5, 6, 7]

        for row in green_rows:
            for col in range(8):
                is_white_square = (row + col) % 2 == 1

                if is_white_square:
                    r_rot, c_rot = self.rotate_square(row, col, rotation)

                    x = ox + c_rot * square_size + square_size / 2.0
                    y = oy + r_rot * square_size + square_size / 2.0
                    z = oz + piece_height / 2.0

                    msg.markers.append(
                        self.make_piece(marker_id, x, y, z, piece_radius, piece_height, "green")
                    )
                    marker_id += 1

        for row in purple_rows:
            for col in range(8):
                is_white_square = (row + col) % 2 == 1

                if is_white_square:
                    r_rot, c_rot = self.rotate_square(row, col, rotation)

                    x = ox + c_rot * square_size + square_size / 2.0
                    y = oy + r_rot * square_size + square_size / 2.0
                    z = oz + piece_height / 2.0

                    msg.markers.append(
                        self.make_piece(marker_id, x, y, z, piece_radius, piece_height, "purple")
                    )
                    marker_id += 1

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ChessboardMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()