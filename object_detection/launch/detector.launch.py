from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    detector_node = Node(package='object_detection',
                       executable='obstacle_detector',
                       output = 'screen',
                       parameters=[{
                        'use_sim_time' : True        
                       }],
                       remappings=[('input_scan','/scan')]
                       )
    
    monitor_node = Node(package='object_detection',
                       executable='obstacle_monitor',
                       output = 'screen',
                       parameters=[{
                        'use_sim_time' : True        
                       }])
    
    ld = LaunchDescription()
    ld.add_action(detector_node)
    ld.add_action(monitor_node)
    
    return ld