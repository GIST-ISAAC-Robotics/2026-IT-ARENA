from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '4', '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-13/race_start_render_v2/run/fixture.sdf'], output="screen"), Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=['/b3_optical/grid2_off@sensor_msgs/msg/Image[gz.msgs.Image', '/b3_optical/grid2_red4@sensor_msgs/msg/Image[gz.msgs.Image', '/b3_optical/grid2_green@sensor_msgs/msg/Image[gz.msgs.Image', '/b3_optical/grid3_off@sensor_msgs/msg/Image[gz.msgs.Image', '/b3_optical/grid3_red4@sensor_msgs/msg/Image[gz.msgs.Image', '/b3_optical/grid3_green@sensor_msgs/msg/Image[gz.msgs.Image'], output="screen")])
