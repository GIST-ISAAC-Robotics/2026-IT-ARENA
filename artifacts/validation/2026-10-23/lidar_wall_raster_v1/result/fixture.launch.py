from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '-v', '4', '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-23/lidar_wall_raster_v1/result/fixture.sdf'], output="screen"), Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=['/audit/wall_raster/straight_corridor@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', '/audit/wall_raster/end_wall@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', '/audit/wall_raster/angled_wall@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'], output="screen")])
