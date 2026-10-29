from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '3', '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-29/signal_spec_1029_v1/render_no_id45_green/fixture.sdf'], output="screen"), Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=['/m5_grid/slot0@sensor_msgs/msg/Image[gz.msgs.Image', '/m5_grid/slot1@sensor_msgs/msg/Image[gz.msgs.Image', '/m5_grid/slot2@sensor_msgs/msg/Image[gz.msgs.Image', '/m5_grid/slot3@sensor_msgs/msg/Image[gz.msgs.Image', '/m5_grid/slot4@sensor_msgs/msg/Image[gz.msgs.Image', '/m5_grid/slot5@sensor_msgs/msg/Image[gz.msgs.Image', '/m5_grid/overview@sensor_msgs/msg/Image[gz.msgs.Image'], output="screen")])
