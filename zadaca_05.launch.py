from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle1'
        ),
        Node(
            package='zadaca_05',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        Node(
            package='zadaca_05',
            executable='goal_broadcaster',
            name='goal_broadcaster',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[]
        )
        
])
