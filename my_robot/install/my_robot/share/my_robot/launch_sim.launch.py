import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'my_robot'

    #Incluyo el anterior launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        os.path.join(
        get_package_share_directory(pkg_name), 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    #Incluyo gazebo

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]), 
    )

    #Spawneo al robot

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_robot'],
                        output = 'screen')
    
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])

