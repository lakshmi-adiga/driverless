from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

# IMPORTANT DISTINCTION: If loading launch file from a package that you have installed through apt-get or through source
# you need to write "launch/{filename}" in the os.path.join
# If package is already in your workspace (i.e. one of the cmrdv_ packages) you don't need to write "launch/" as I've done here

def generate_launch_description():
    ld = LaunchDescription()
    
    velodyne = Node(
        package='perceptions',
        executable='stereo_vision'
    )

    zed = Node(
        package='perceptions',
        executable='lidar'
    )

    lidar = Node(
        package='perceptions',
        executable='lidar'
    )

    stereo = Node(
        package='perceptions',
        executable='stereo_vision'
    )

    midline = Node(
        package='planning',
        executable='midline'
    )

    ld.add_action(data_collection)
    ld.add_action(data_collection)
    ld.add_action(lidar)
    ld.add_action(stereo)
    ld.add_action(midline)



    return ld
