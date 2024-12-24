from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='improved_artificial_potential_field_algorithm',
            executable='improved_apf_node',
            name='improved_apf_node',
            output='screen',
            parameters=[
                {'planner_topic_': '/planned_path'},
                {'obstacle_influence_distance_': 5.0},
                {'field_of_view_angle_degrees_': 180.0},
                {'lookahead_distance_': 2.0},
                {'R_avoid_factor_': 1.0},
                {'K_F_': 1.0},
                {'K_delta_': 1.0},
                {'wheel_base_': 0.3302},
                {'do_': 0.1},
                {'sliding_window_length_': 5},
                {'goal_threshold_distance_': 0.5},
                {'slowest_speed_': 1.0},
                {'mid_speed_': 2.5},
                {'max_speed_': 5.0},
            ],
        ),
    ])
