#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


# Environment dimensions, metres
FRONT_DIST = 0.40
BACK_DIST = 1.00
RIGHT_DIST = 1.00
LEFT_DIST = 0.25
CEILING_HT = 1.20
WALL_THICKNESS = 0.02

# Full workspace floor height
FLOOR_HEIGHT = 0.00

# Checkerboard-only raised floor
BOARD_X = -0.60
BOARD_Y = 0.25
BOARD_Z = 0.00
BOARD_SIZE = 0.40          # 8 squares x 0.05 m
BOARD_CLEARANCE_Z = 0.025   # raised safety plane above board
BOARD_SAFETY_MARGIN = 0.03

X_SPAN = FRONT_DIST + BACK_DIST
Y_SPAN = RIGHT_DIST + LEFT_DIST
Z_SPAN = CEILING_HT

X_CENTRE = (FRONT_DIST - BACK_DIST) / 2.0
Y_CENTRE = (RIGHT_DIST - LEFT_DIST) / 2.0
Z_CENTRE = CEILING_HT / 2.0


class BoundingBoxNode(Node):
    def __init__(self):
        super().__init__("bounding_box_node")

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
        scene = PlanningScene()
        scene.is_diff = True

        T = WALL_THICKNESS

        # Full workspace floor
        scene.world.collision_objects.append(self.make_box(
            name="floor",
            size_xyz=(X_SPAN + 2*T, Y_SPAN + 2*T, T),
            pos_xyz=(X_CENTRE, Y_CENTRE, FLOOR_HEIGHT - T/2.0)
        ))

        # Checkerboard-only raised safety plane
        scene.world.collision_objects.append(self.make_box(
            name="checkerboard_raised_floor",
            size_xyz=(
                BOARD_SIZE + 2*BOARD_SAFETY_MARGIN,
                BOARD_SIZE + 2*BOARD_SAFETY_MARGIN,
                T
            ),
            pos_xyz=(
                BOARD_X + BOARD_SIZE/2.0,
                BOARD_Y + BOARD_SIZE/2.0,
                BOARD_Z + BOARD_CLEARANCE_Z - T/2.0
            )
        ))

        # Ceiling
        scene.world.collision_objects.append(self.make_box(
            name="ceiling",
            size_xyz=(X_SPAN + 2*T, Y_SPAN + 2*T, T),
            pos_xyz=(X_CENTRE, Y_CENTRE, CEILING_HT + T/2.0)
        ))

        # Front wall +X
        scene.world.collision_objects.append(self.make_box(
            name="front_wall",
            size_xyz=(T, Y_SPAN + 2*T, Z_SPAN + 2*T),
            pos_xyz=(FRONT_DIST + T/2.0, Y_CENTRE, Z_CENTRE)
        ))

        # Back wall -X
        scene.world.collision_objects.append(self.make_box(
            name="back_wall",
            size_xyz=(T, Y_SPAN + 2*T, Z_SPAN + 2*T),
            pos_xyz=(-BACK_DIST - T/2.0, Y_CENTRE, Z_CENTRE)
        ))

        # Right wall +Y
        scene.world.collision_objects.append(self.make_box(
            name="right_wall",
            size_xyz=(X_SPAN + 2*T, T, Z_SPAN + 2*T),
            pos_xyz=(X_CENTRE, RIGHT_DIST + T/2.0, Z_CENTRE)
        ))

        # Left wall -Y
        scene.world.collision_objects.append(self.make_box(
            name="left_wall",
            size_xyz=(X_SPAN + 2*T, T, Z_SPAN + 2*T),
            pos_xyz=(X_CENTRE, -LEFT_DIST - T/2.0, Z_CENTRE)
        ))

        self.pub.publish(scene)

        self.get_logger().info(
            "\nPlanning scene published!\n"
            f"  Floor:                    z = {FLOOR_HEIGHT:.2f} m\n"
            f"  Checkerboard raised area: z = {BOARD_Z + BOARD_CLEARANCE_Z:.2f} m\n"
            f"  Checkerboard centre:      x = {BOARD_X + BOARD_SIZE/2.0:.2f}, "
            f"y = {BOARD_Y + BOARD_SIZE/2.0:.2f}\n"
            f"  Ceiling:                  z = {CEILING_HT:.2f} m\n"
        )

        self.timer.cancel()


def main():
    rclpy.init()
    node = BoundingBoxNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()