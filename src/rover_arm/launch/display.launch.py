from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_path = get_package_share_directory('rover_arm')
    urdf_file = os.path.join(pkg_path, 'urdf', 'rover_arm.urdf')
    rviz_config = os.path.join(pkg_path, 'urdf.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch argument to select backend — defaults to sim
    # To use CAN backend (when ready): ros2 launch rover_arm display.launch.py backend:=can
    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='sim',
        description='HAL backend to use: sim or can'
    )

    # Publishes TF transforms from the URDF joint states.
    # Reads robot_description parameter and /arm/joint_states topic.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        remappings=[('/joint_states', '/arm/joint_states')]  # this is critical
    )

    # arm_control_node — subscribes to /arm/cmd_joint, drives the HAL,
    # and publishes /arm/joint_states which robot_state_publisher reads.
    arm_control_node = Node(
        package='rover_arm',
        executable='arm_control_node',
        name='arm_control_node',
        parameters=[{
            'backend': LaunchConfiguration('backend'),
            'publish_rate_hz': 20.0,
            'robot_description': robot_description,  # ADD THIS
        }],
        output='screen'
    )

    # RViz2 — visualises the robot model and TF frames
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        backend_arg,
        robot_state_publisher,
        arm_control_node,
        rviz,
    ])