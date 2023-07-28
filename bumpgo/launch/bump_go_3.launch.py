from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    bumpgo_node = Node(package='bumpgo',
                       executable='bump_go_3',
                       output = 'screen',
                       parameters=[{
                        'use_sim_time' : True        
                       }],
                       remappings=[
                        ('input_scan','/scan'),
                        ('output_vel', '/cmd_vel')
                       ])
    
    ld = LaunchDescription()
    ld.add_action(bumpgo_node)
    
    return ld