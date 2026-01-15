from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def load_file(path: str) -> str:
    with open(path, "r") as f:
        return f.read()

def launch_setup(context, *args, **kwargs):
    # Resolve launch args at runtime
    model_path   = LaunchConfiguration("model").perform(context)
    start_rviz   = LaunchConfiguration("start_rviz")  # keep as Substitution for IfCondition
    robot_desc   = load_file(model_path)

    nodes = [
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_desc}],
        ),
        # Only start RViz if explicitly requested
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(start_rviz),
            # optionally: arguments=["-d", LaunchConfiguration("rviz_config")]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_pit_static_tf",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "world",
                "--child-frame-id", "pit_base_link",
    ],
),
    ]
    return nodes

def generate_launch_description():
    pkg_share = get_package_share_directory("new_link_arm_full")
    default_model = os.path.join(pkg_share, "urdf", "robot_pit_end_effector.urdf")

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value=default_model,
            description="Absolute path to robot URDF file",
        ),
        # New toggle: RViz is OFF by default
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Set true to launch RViz",
        ),
        OpaqueFunction(function=launch_setup),
    ])
