from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'turtlebot3',
                 '-database', 'turtlebot3_world'],
            output='screen'
        ),
        
        Node(
            package='patrol_behavior_tree',
            executable='patrol_bt_node',
            name='patrol_bt',
            output='screen'
        )
    ])