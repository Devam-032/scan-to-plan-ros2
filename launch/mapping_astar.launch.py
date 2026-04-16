import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='2.0',
        description='A* goal X position in odom frame'
    )
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='2.0',
        description='A* goal Y position in odom frame'
    )

    # Default RViz config — falls back gracefully if file doesn't exist
    default_rviz_config = os.path.join(
        get_package_share_directory('turtlebot3_tuts'),
        '.rviz2',
        'mapping.rviz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz2 config file'
    )

    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir,
                         'launch',
                         'turtlebot3_world.launch.py')
        )
    )

    your_node = Node(
        package='turtlebot3_tuts',
        executable='scan_to_cartesian',
        name='scan_to_cartesian',
        output='screen',
        parameters=[{
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    delayed_nodes = TimerAction(
        period=5.0,
        actions=[your_node, rviz_node]
    )

    return LaunchDescription([
        goal_x_arg,
        goal_y_arg,
        rviz_config_arg,
        turtlebot3_world,
        delayed_nodes,
    ])
