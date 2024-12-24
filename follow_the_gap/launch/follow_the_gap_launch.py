# follow_the_gap_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_the_gap',
            executable='follow_the_gap_node',
            name='follow_the_gap_node',
            output='screen',
            parameters=[
                {'obstacle_distance_threshold_': 6.0},
                {'field_of_view_angle_degrees_': 180.0},
                {'safety_bubble_radius_': 0.5},
                {'sliding_window_length_': 5},
                {'safety_angle_': 20.0},
                {'buffer_angle_scaling_factor_': 6.0},
            ],
        ),
    ])

