#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


class BoundingBoxNode(Node):
    def __init__(self):
        super().__init__("bounding_box_node")

        # Environment dimensions, metres
        self.declare_parameter("front_dist", 0.50)
        self.declare_parameter("back_dist", 0.50)
        self.declare_parameter("right_dist", 1.00)
        self.declare_parameter("left_dist", 0.40)
        self.declare_parameter("ceiling_height", 1.20)
        self.declare_parameter("wall_thickness", 0.02)

        # Full workspace floor
        self.declare_parameter("floor_height", 0.00)

        # Checkerboard raised safety plane
        self.declare_parameter("board_x", -0.20)
        self.declare_parameter("board_y", 0.35)
        self.declare_parameter("board_z", 0.00)
        self.declare_parameter("board_size", 0.40)
        self.declare_parameter("board_clearance_z", 0.025)
        self.declare_parameter("board_safety_margin", 0.03)

        self.pub = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )

        self.timer = self.create_timer(1.0, self.publish_scene)

        self.get_logger().info(
            "BoundingBoxNode started — publishing in 1 s..."
        )

    def make_box(self, name, size_xyz, pos_xyz, frame_id="base_link"):
        obj = CollisionObject()
        obj.header = Header()
        obj.header.frame_id = frame_id
        obj.id = name

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size_xyz)

        pose = Pose()
        pose.position.x = float(pos_xyz[0])
        pose.position.y = float(pos_xyz[1])
        pose.position.z = float(pos_xyz[2])
        pose.orientation.w = 1.0

        obj.primitives = [box]
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        return obj

    def publish_scene(self):
        front_dist = float(self.get_parameter("front_dist").value)
        back_dist = float(self.get_parameter("back_dist").value)
        right_dist = float(self.get_parameter("right_dist").value)
        left_dist = float(self.get_parameter("left_dist").value)
        ceiling_ht = float(self.get_parameter("ceiling_height").value)
        wall_thickness = float(self.get_parameter("wall_thickness").value)

        floor_height = float(self.get_parameter("floor_height").value)

        board_x = float(self.get_parameter("board_x").value)
        board_y = float(self.get_parameter("board_y").value)
        board_z = float(self.get_parameter("board_z").value)
        board_size = float(self.get_parameter("board_size").value)
        board_clearance_z = float(self.get_parameter("board_clearance_z").value)
        board_safety_margin = float(self.get_parameter("board_safety_margin").value)

        T = wall_thickness

        x_span = front_dist + back_dist
        y_span = right_dist + left_dist
        z_span = ceiling_ht

        x_centre = (front_dist - back_dist) / 2.0
        y_centre = (right_dist - left_dist) / 2.0
        z_centre = ceiling_ht / 2.0

        scene = PlanningScene()
        scene.is_diff = True

        # Full workspace floor
        scene.world.collision_objects.append(self.make_box(
            name="floor",
            size_xyz=(x_span + 2*T, y_span + 2*T, T),
            pos_xyz=(x_centre, y_centre, floor_height - T/2.0)
        ))

        # Checkerboard solid raised block
        # Extends from floor (z=0) to board_z + board_clearance_z

        board_top_z = board_z + board_clearance_z
        board_thickness = board_top_z

        scene.world.collision_objects.append(self.make_box(
            name="checkerboard_raised_floor",
            size_xyz=(
                board_size + 2*board_safety_margin,
                board_size + 2*board_safety_margin,
                board_thickness
            ),
            pos_xyz=(
                board_x + board_size/2.0,
                board_y + board_size/2.0,
                board_thickness/2.0
            )
        ))

        # Ceiling
        scene.world.collision_objects.append(self.make_box(
            name="ceiling",
            size_xyz=(x_span + 2*T, y_span + 2*T, T),
            pos_xyz=(x_centre, y_centre, ceiling_ht + T/2.0)
        ))

        # Front wall +X
        scene.world.collision_objects.append(self.make_box(
            name="front_wall",
            size_xyz=(T, y_span + 2*T, z_span + 2*T),
            pos_xyz=(front_dist + T/2.0, y_centre, z_centre)
        ))

        # Back wall -X
        scene.world.collision_objects.append(self.make_box(
            name="back_wall",
            size_xyz=(T, y_span + 2*T, z_span + 2*T),
            pos_xyz=(-back_dist - T/2.0, y_centre, z_centre)
        ))

        # Right wall +Y
        scene.world.collision_objects.append(self.make_box(
            name="right_wall",
            size_xyz=(x_span + 2*T, T, z_span + 2*T),
            pos_xyz=(x_centre, right_dist + T/2.0, z_centre)
        ))

        # Left wall -Y
        scene.world.collision_objects.append(self.make_box(
            name="left_wall",
            size_xyz=(x_span + 2*T, T, z_span + 2*T),
            pos_xyz=(x_centre, -left_dist - T/2.0, z_centre)
        ))

        self.pub.publish(scene)

        self.get_logger().info(
            "\nPlanning scene published!\n"
            f"  Floor:                    z = {floor_height:.3f} m\n"
            f"  Checkerboard raised area: z = {board_z + board_clearance_z:.3f} m\n"
            f"  Checkerboard origin:      x = {board_x:.3f}, y = {board_y:.3f}, z = {board_z:.3f}\n"
            f"  Checkerboard centre:      x = {board_x + board_size/2.0:.3f}, "
            f"y = {board_y + board_size/2.0:.3f}\n"
            f"  Front/back dists:         +X {front_dist:.3f}, -X {back_dist:.3f}\n"
            f"  Right/left dists:         +Y {right_dist:.3f}, -Y {left_dist:.3f}\n"
            f"  Ceiling:                  z = {ceiling_ht:.3f} m\n"
        )

        self.timer.cancel()


def main():
    rclpy.init()
    node = BoundingBoxNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()