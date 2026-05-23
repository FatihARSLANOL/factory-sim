# cd ~/ros2_factory_ws
# ros2 launch factory_sim factory.launch.py


from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='factory_sim',
            executable='environment',
            name='environment_node',
            output='screen'
        ),
        Node(
            package='factory_sim',
            executable='robot',
            name='robot_node',
            output='screen'
        ),
        Node(
            package='factory_sim',
            executable='visualizer',
            name='visualizer_node',
            output='screen'
        ),
        Node(
            package='factory_sim',
            executable='rviz_publisher_node',
            name='rviz_publisher_node',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', '/home/fatih/ros2_factory_ws/src/factory_sim/library_robot.rviz'],
            output='screen'
        ),
    ])