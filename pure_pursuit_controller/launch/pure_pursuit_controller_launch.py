import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    window_size_arg = DeclareLaunchArgument('window_size_', default_value='8')
    beta_max_arg = DeclareLaunchArgument('beta_max_', default_value='15')
    alpha_max_arg = DeclareLaunchArgument('alpha_max_', default_value='0.5')
    path_planner_arg = DeclareLaunchArgument('path_planner', default_value='A_star')
    controller_type_arg = DeclareLaunchArgument('controller_type', default_value='default')

    # Parameters
    window_size_ = LaunchConfiguration('window_size_')
    beta_max_ = LaunchConfiguration('beta_max_')
    alpha_max_ = LaunchConfiguration('alpha_max_')
    lookahead_distance_ = '0.9'
    slowest_speed_ = '2'
    mid_speed_ = '3'
    max_speed_ = '4'

    # Function to include nodes conditionally
    def include_nodes(context, *args, **kwargs):
        path_planner = context.launch_configurations.get('path_planner')
        controller_type = context.launch_configurations.get('controller_type')

        nodes = []

        # Common parameters
        common_params = {
            'window_size_': window_size_,
            'beta_max_': beta_max_,
            'alpha_max_': alpha_max_,
            'lookahead_distance_': lookahead_distance_,
            'slowest_speed_': slowest_speed_,
            'mid_speed_': mid_speed_,
            'max_speed_': max_speed_,
        }

        # Choose the controller node to launch based on the controller_type argument
        if controller_type == 'default':
            nodes.append(Node(
                package='pure_pursuit_controller',
                executable='pure_pursuit_node',
                name='pure_pursuit_controller_node',
                output='screen',
                parameters=[common_params]
            ))
        elif controller_type == 'with_delay':
            nodes.append(Node(
                package='pure_pursuit_controller',
                executable='pure_pursuit_node_with_delay',
                name='pure_pursuit_controller_node_with_delay',
                output='screen',
                parameters=[common_params]
            ))
        elif controller_type == 'lingering':
            nodes.append(Node(
                package='pure_pursuit_controller',
                executable='pure_pursuit_lingering_node',
                name='pure_pursuit_controller_node_with_lingering_lookahead_point',
                output='screen',
                parameters=[common_params]
            ))
        else:
            # Default to the basic controller if no valid controller_type is provided
            nodes.append(Node(
                package='pure_pursuit_controller',
                executable='pure_pursuit_node',
                name='pure_pursuit_controller_node',
                output='screen',
                parameters=[common_params]
            ))

        # Planner Nodes
        if path_planner == 'dijkstra':
            nodes.append(Node(
                package='motion_planning_dijkstra',
                executable='motion_planning_dijkstra_node',
                name='motion_planning_dijkstra_node',
                output='screen',
                parameters=[{'planner_topic': '/dijkstra_shortest_path'}]
            ))
        elif path_planner == 'A_star':
            nodes.append(Node(
                package='motion_planning_A_star',
                executable='motion_planning_A_star_node',
                name='motion_planning_A_star_node',
                output='screen',
                parameters=[{'planner_topic': '/A_star_shortest_path'}]
            ))
        elif path_planner == 'RRT':
            nodes.append(Node(
                package='motion_planning_RRT',
                executable='motion_planning_RRT_node',
                name='motion_planning_RRT_node',
                output='screen',
                parameters=[{'planner_topic': '/rrt_path'}]
            ))
        elif path_planner == 'RRT_star':
            nodes.append(Node(
                package='motion_planning_RRT_star',
                executable='motion_planning_RRT_star_node',
                name='motion_planning_RRT_star_node',
                output='screen',
                parameters=[{'planner_topic': '/rrt_star_path'}]
            ))
        else:
            # If no valid path_planner is specified, default to A* planner
            nodes.append(Node(
                package='motion_planning_A_star',
                executable='motion_planning_A_star_node',
                name='motion_planning_A_star_node',
                output='screen',
                parameters=[{'planner_topic': '/A_star_shortest_path'}]
            ))

        return nodes

    return LaunchDescription([
        window_size_arg,
        beta_max_arg,
        alpha_max_arg,
        path_planner_arg,
        controller_type_arg,
        OpaqueFunction(function=include_nodes)
    ])

