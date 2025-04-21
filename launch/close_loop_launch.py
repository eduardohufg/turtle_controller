from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    sp_node2 = Node(package='turtlesim',
                       executable='turtlesim_node',
                       name="turtlesim",
                       )
        
    motor_node1 = Node(package='turtle_controller',
                       executable='path_generator',
                       name="path_gen",
                       )
    
    sp_node1 = Node(package='turtle_controller',
                       executable='pz_close',
                       name="pz_close",
                       )
    
    
    
    l_d = LaunchDescription([motor_node1, sp_node1, sp_node2])

    return l_d