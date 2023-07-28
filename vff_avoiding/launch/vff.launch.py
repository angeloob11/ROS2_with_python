from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    vff_node = Node(package='vff_avoiding',
                       executable='vff_avoiding',
                       output = 'screen',
                       parameters=[{
                        'use_sim_time' : True        
                       }],
                       remappings=[
                        ('input_scan','/scan'),
                        ('output_vel', '/cmd_vel')
                       ])
    
    ld = LaunchDescription()
    ld.add_action(vff_node)
    
    return ld