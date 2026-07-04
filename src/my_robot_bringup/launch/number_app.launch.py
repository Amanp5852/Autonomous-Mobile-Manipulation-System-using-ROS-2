from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    number_publisher = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="my_number_publisher",
        remappings=[("/number", "/my_number")],
        parameters=[
            {"number": 12},
            {"timer_period": 1.3}
        ]
    )
    
    number_subscirber = Node(
        package="my_py_pkg",
        executable="number_subscirber"
    )
    ld.add_action(number_publisher)
    ld.add_action(number_subscirber)
    
    return ld
