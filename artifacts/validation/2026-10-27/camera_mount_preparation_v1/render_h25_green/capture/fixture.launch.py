from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '3', '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-27/camera_mount_preparation_v1/render_h25_green/fixture.sdf'], output="screen"), Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=['/m5_grid/overview@sensor_msgs/msg/Image[gz.msgs.Image', '/m5_grid/rear_20cm@sensor_msgs/msg/Image[gz.msgs.Image', '/m5_grid/rear_30cm@sensor_msgs/msg/Image[gz.msgs.Image'], output="screen")])
