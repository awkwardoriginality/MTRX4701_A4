#!/usr/bin/env python3
"""
UR5e Workspace Bounding Box Node
==================================
Publishes collision objects to MoveIt's planning scene to confine
the robot within the real-world environment limits.

Limits from robot base_link origin (corrected axes):
  Front wall (end effector direction, +X) : 100 cm
  Back wall  (behind robot,          -X)  :  40 cm
  Right wall (right of end effector, +Y)  : 100 cm
  Left wall  (left of end effector,  -Y)  :  50 cm
  Ceiling    (above robot,           +Z)  : 120 cm
  Floor      (below base,            -Z)  :   5 cm slab

Usage:
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/my-UR5e-robotiq-hande-workspace/install/setup.bash
  python3 bounding_box_node.py

Run AFTER your RViz2 and MoveIt terminals are fully started.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


# ──────────────────────────────────────────────
# Environment dimensions (metres)
# Adjust these if your setup changes
# ──────────────────────────────────────────────
FRONT_DIST  = 0.40   # +X  (end effector faces this direction)
BACK_DIST   = 1.00   # -X  (behind robot)
RIGHT_DIST  = 1.00   # +Y  (right of end effector)
LEFT_DIST   = 0.25   # -Y  (left of end effector)
CEILING_HT  = 1.20   # +Z  (above base_link)
WALL_THICKNESS = 0.02  # thickness of each collision slab (5 cm)

# Total span used to size each wall's face
X_SPAN = FRONT_DIST + BACK_DIST    # 1.40 m
Y_SPAN = RIGHT_DIST + LEFT_DIST    # 1.50 m
Z_SPAN = CEILING_HT                # 1.20 m

# Centre offsets from base_link
X_CENTRE = (FRONT_DIST - BACK_DIST) / 2.0    #  0.30 m
Y_CENTRE = (RIGHT_DIST - LEFT_DIST) / 2.0    #  0.25 m
Z_CENTRE = CEILING_HT / 2.0                   #  0.60 m


class BoundingBoxNode(Node):
    def __init__(self):
        super().__init__('bounding_box_node')
        self.pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10
        )
        self.timer = self.create_timer(1.0, self.publish_scene)
        self.get_logger().info('BoundingBoxNode node started — publishing in 1 s...')

    def make_box(self, name, size_xyz, pos_xyz, frame_id='base_link'):
        """Return a box CollisionObject centred at pos_xyz."""
        obj = CollisionObject()
        obj.header = Header()
        obj.header.frame_id = frame_id
        obj.id = name

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size_xyz)   # [x, y, z] in metres

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

        # ── Floor slab (just below base_link, z=0) ──────────────────
        scene.world.collision_objects.append(self.make_box(
            name     = 'floor',
            size_xyz = (X_SPAN + 2*T, Y_SPAN + 2*T, T),
            pos_xyz  = (X_CENTRE, Y_CENTRE, -T / 2.0)
        ))

        # ── Ceiling ──────────────────────────────────────────────────
        scene.world.collision_objects.append(self.make_box(
            name     = 'ceiling',
            size_xyz = (X_SPAN + 2*T, Y_SPAN + 2*T, T),
            pos_xyz  = (X_CENTRE, Y_CENTRE, CEILING_HT + T / 2.0)
        ))

        # ── Front wall (+X, 100 cm — end effector direction) ─────────
        scene.world.collision_objects.append(self.make_box(
            name     = 'front_wall',
            size_xyz = (T, Y_SPAN + 2*T, Z_SPAN + 2*T),
            pos_xyz  = (FRONT_DIST + T / 2.0, Y_CENTRE, Z_CENTRE)
        ))

        # ── Back wall (-X, 40 cm — behind robot) ─────────────────────
        scene.world.collision_objects.append(self.make_box(
            name     = 'back_wall',
            size_xyz = (T, Y_SPAN + 2*T, Z_SPAN + 2*T),
            pos_xyz  = (-BACK_DIST - T / 2.0, Y_CENTRE, Z_CENTRE)
        ))

        # ── Right wall (+Y, 100 cm — right of end effector) ──────────
        scene.world.collision_objects.append(self.make_box(
            name     = 'right_wall',
            size_xyz = (X_SPAN + 2*T, T, Z_SPAN + 2*T),
            pos_xyz  = (X_CENTRE, RIGHT_DIST + T / 2.0, Z_CENTRE)
        ))

        # ── Left wall (-Y, 50 cm — left of end effector) ─────────────
        scene.world.collision_objects.append(self.make_box(
            name     = 'left_wall',
            size_xyz = (X_SPAN + 2*T, T, Z_SPAN + 2*T),
            pos_xyz  = (X_CENTRE, -LEFT_DIST - T / 2.0, Z_CENTRE)
        ))

        self.pub.publish(scene)
        self.get_logger().info(
            'Planning scene published!\n'
            '  front_wall: +X 100 cm (end effector direction)\n'
            '  back_wall:  -X  40 cm\n'
            '  right_wall: +Y 100 cm\n'
            '  left_wall:  -Y  50 cm\n'
            '  ceiling:    +Z 120 cm\n'
            '  floor:       0 cm (base mount level)\n'
            '  Check the "Scene Objects" tab in your MoveIt RViz2 window.'
        )
        self.timer.cancel()


def main():
    rclpy.init()
    node = BoundingBoxNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()