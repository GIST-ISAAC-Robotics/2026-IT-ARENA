from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
    return LaunchDescription([ExecuteProcess(cmd=["gz","sim","-s","-r","--headless-rendering",'/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/sparse_upper_r341_v1/gpu_probe/world.sdf'], output="screen")])
