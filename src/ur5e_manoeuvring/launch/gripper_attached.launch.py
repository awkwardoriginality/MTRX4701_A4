from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur5e_manoeuvring"),
            "urdf",
            "hande_attached_to_tool0.urdf.xacro",
        ]),
    ])

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str,
        )
    }

    robot_controllers = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("robotiq_hande_driver"),
            "bringup",
            "config",
            "hande_controller.yaml",
        ]),
        allow_substs=True,
    )

    gripper_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="gripper",
        parameters=[robot_description],
        output="both",
    )

    gripper_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="gripper",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/gripper/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/gripper/controller_manager",
        ],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_action_controller",
            "--controller-manager",
            "/gripper/controller_manager",
        ],
        output="screen",
    )

    tf_prefix_arg = DeclareLaunchArgument(
    "tf_prefix",
    default_value="",
    description="Prefix for gripper joints/links",
)

    return LaunchDescription([
        tf_prefix_arg,
        gripper_state_publisher,
        gripper_control_node,
        joint_state_broadcaster_spawner,
        gripper_controller_spawner,
    ])