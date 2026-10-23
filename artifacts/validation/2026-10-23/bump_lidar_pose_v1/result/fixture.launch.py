from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '4', '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-23/bump_lidar_pose_v1/result/fixture.sdf'], output="screen"), Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=['/audit/bump_pose/level@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', '/audit/bump_pose/nose_down@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', '/audit/bump_pose/nose_up@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'], output="screen")])
