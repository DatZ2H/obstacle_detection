import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/depth/color/points',
        description='Input point cloud topic'
    )

    # Get path to package
    package_dir = get_package_share_directory('obstacle_detection')

    # Define node
    obstacle_detection_node = Node(
        package='obstacle_detection',
        executable='obstacle_detection_node',
        name='obstacle_detection_node',
        parameters=[{'input_topic': LaunchConfiguration('input_topic')}]
    )

    # Create launch description
    ld = LaunchDescription()

    # Add actions to launch description
    ld.add_action(input_topic)
    ld.add_action(obstacle_detection_node)

    return ld
