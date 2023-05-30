from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node





def generate_launch_description():
    DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Target port name.'
      ),
    DeclareLaunchArgument(
        'frame_id', default_value='ais_link',
        description='Target frame name.'
      ),
    return LaunchDescription([
        Node(
            package='ros2_ais',
            executable='ros_daisy_node.py',
            name='AIS_node',
            output='screen',
            respawn= True,
            parameters=[
                {'serial_port': LaunchConfiguration('serial_port')},
                {'frame_id': LaunchConfiguration('frame_id')},
                {'frequency': 10},
                {'baudrate': 38400}
            ]
        )
    ])