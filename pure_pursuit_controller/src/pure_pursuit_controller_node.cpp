/* 
@file: pure_pursuit_controller_node.cpp
@description: Pure Pursuit Controller Node converted to ROS 2. Subscribes to the path planner topic loaded in the parameter server to get waypoints.
*/

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <algorithm>
#include <numeric>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

const float PI = 3.14159;

class PurePursuitController : public rclcpp::Node
{
private:
    // Publishers and Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr car_pose_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_command_publisher_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Member Variables
    nav_msgs::msg::Path path_to_track_map_frame_;
    std::string planner_topic_;
    float lookahead_distance_;
    int path_seq_;
    int path_size_;
    float max_steering_angle_ = 0.4189;   
    float mid_speed_;        
    float slowest_speed_;
    float max_speed_;
    bool path_detected_;
    std::pair<float, float> goal_point_;
    float goal_threshold_distance_;

public:
    // Constructor
    PurePursuitController();

private:
    // Helper Functions
    int goal_closeness(const std::pair<float, float>& car_position);
    float dist(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2);
    void stop_car();

    // Callback Functions
    void path_subscriber_callback(const nav_msgs::msg::Path::SharedPtr planner_path);
    void car_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr car_pose);
};

// Constructor Implementation
PurePursuitController::PurePursuitController()
    : Node("pure_pursuit_controller_node")
{
    // Declare parameters with default values
    this->declare_parameter<std::string>("planner_topic", "/planned_path");
    this->declare_parameter<float>("lookahead_distance_", 2.0);
    this->declare_parameter<float>("slowest_speed_", 1.0);
    this->declare_parameter<float>("mid_speed_", 2.5);
    this->declare_parameter<float>("max_speed_", 5.0);
    this->declare_parameter<float>("goal_threshold_distance_", 0.5);

    // Get parameters
    planner_topic_ = this->get_parameter("planner_topic").as_string();
    lookahead_distance_ = this->get_parameter("lookahead_distance_").as_double();
    slowest_speed_ = this->get_parameter("slowest_speed_").as_double();
    mid_speed_ = this->get_parameter("mid_speed_").as_double();
    max_speed_ = this->get_parameter("max_speed_").as_double();
    goal_threshold_distance_ = this->get_parameter("goal_threshold_distance_").as_double();

    // Initialize subscribers and publishers
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        planner_topic_, 1, std::bind(&PurePursuitController::path_subscriber_callback, this, std::placeholders::_1));
    car_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/gt_pose", 1, std::bind(&PurePursuitController::car_pose_callback, this, std::placeholders::_1));
    drive_command_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", 1);

    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Wait for the TF buffer to fill
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Initialize other member variables
    path_seq_ = 0;
    path_detected_ = false;

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller node initialized");
}

// Helper Functions Implementations
int PurePursuitController::goal_closeness(const std::pair<float, float>& car_position)
{
    float dx = car_position.first - goal_point_.first;
    float dy = car_position.second - goal_point_.second;
    float distance = sqrt((dx * dx) + (dy * dy));
    if (distance <= goal_threshold_distance_)
    {
        return 1;
    }
    else if (distance <= 2.5)
    {
        return 2;
    }
    else
    {
        return 3;
    }
}

float PurePursuitController::dist(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2)
{
    float dx = pose2.pose.position.x - pose1.pose.position.x;
    float dy = pose2.pose.position.y - pose1.pose.position.y;
    return sqrt((dx * dx) + (dy * dy));
}

void PurePursuitController::stop_car()
{
    ackermann_msgs::msg::AckermannDriveStamped drive_command;
    drive_command.header.stamp = this->now();
    drive_command.header.frame_id = "base_link";
    drive_command.drive.steering_angle = 0.0;
    drive_command.drive.speed = 0.0;
    drive_command_publisher_->publish(drive_command);
}

// Callback Functions Implementations
void PurePursuitController::path_subscriber_callback(const nav_msgs::msg::Path::SharedPtr planner_path)
{
    // Fetching the path published by planner and saving as class attribute.
    path_size_ = planner_path->poses.size();
    RCLCPP_INFO(this->get_logger(), "Path size from %s is: %d", planner_topic_.c_str(), path_size_);

    std::vector<geometry_msgs::msg::PoseStamped> temp_poses(path_size_);
    std::vector<geometry_msgs::msg::PoseStamped> dense_temp_poses;

    for (int i = 0; i < path_size_; i++)
    {
        temp_poses[i].pose.position.x = planner_path->poses[i].pose.position.x;
        temp_poses[i].pose.position.y = planner_path->poses[i].pose.position.y;
        temp_poses[i].pose.position.z = planner_path->poses[i].pose.position.z;
        temp_poses[i].header.stamp = this->now();
        temp_poses[i].header.frame_id = "map";
    }

    dense_temp_poses.push_back(temp_poses[0]);
    float point_spacing = 0.1;

    for (int i = 1; i < path_size_; i++)
    {
        while (dist(dense_temp_poses.back(), temp_poses[i]) > point_spacing)
        {
            float DY = temp_poses[i].pose.position.y - dense_temp_poses.back().pose.position.y;
            float DX = temp_poses[i].pose.position.x - dense_temp_poses.back().pose.position.x;
            float t = point_spacing / (sqrt((DX * DX) + (DY * DY)));
            geometry_msgs::msg::PoseStamped pose_to_add;
            pose_to_add.pose.position.x = (1 - t) * dense_temp_poses.back().pose.position.x + t * temp_poses[i].pose.position.x;
            pose_to_add.pose.position.y = (1 - t) * dense_temp_poses.back().pose.position.y + t * temp_poses[i].pose.position.y;
            pose_to_add.pose.position.z = 0.0;
            pose_to_add.header.stamp = this->now();
            pose_to_add.header.frame_id = "map";
            dense_temp_poses.push_back(pose_to_add);
        }
    }

    geometry_msgs::msg::PoseStamped final_point;
    final_point.pose.position.x = planner_path->poses.back().pose.position.x;
    final_point.pose.position.y = planner_path->poses.back().pose.position.y;
    final_point.pose.position.z = 0.0;
    final_point.header.stamp = this->now();
    final_point.header.frame_id = "map";
    dense_temp_poses.push_back(final_point);

    path_to_track_map_frame_.header.frame_id = "map";
    path_to_track_map_frame_.poses = dense_temp_poses;
    goal_point_.first = planner_path->poses[(path_size_ - 1)].pose.position.x;
    goal_point_.second = planner_path->poses[(path_size_ - 1)].pose.position.y;
    path_size_ = path_to_track_map_frame_.poses.size();

    RCLCPP_INFO(this->get_logger(), "Dense path size is: %d", path_size_);
    path_detected_ = true;
}

void PurePursuitController::car_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr car_pose)
{
    // Pure pursuit algorithm implemented here.
    if (path_detected_)
    {
        double car_x = car_pose->pose.position.x;
        double car_y = car_pose->pose.position.y;
        double target_point_x = 0.0;
        double target_point_y = 0.0;
        double target_point_dist = 0.0;

        geometry_msgs::msg::TransformStamped map_to_car_frame;

        // Wait for the transform to be available
        if (!tf_buffer_->canTransform("base_link", "map", tf2::TimePointZero))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for transform between 'base_link' and 'map'");
            return;
        }

        try
        {
            // Get the latest transform
            map_to_car_frame = tf_buffer_->lookupTransform("base_link", "map", tf2::TimePointZero);
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
            return;
        }

        // Going over all path points in the path beyond the last path_seq_
        bool intersection_found = false;
        geometry_msgs::msg::PointStamped target_point_map_frame;
        geometry_msgs::msg::PointStamped target_point_car_frame;

        for (int i = path_seq_; i < (path_size_ - 1); i++)
        {
            double t1;
            double t2;
            double ex = path_to_track_map_frame_.poses[i].pose.position.x;
            double ey = path_to_track_map_frame_.poses[i].pose.position.y;
            double lx = path_to_track_map_frame_.poses[i + 1].pose.position.x;
            double ly = path_to_track_map_frame_.poses[i + 1].pose.position.y;
            double dx = lx - ex;
            double dy = ly - ey;
            double fx = ex - car_x;
            double fy = ey - car_y;
            double a = dx * dx + dy * dy;
            double b = 2 * (fx * dx + fy * dy);
            double c = (fx * fx + fy * fy) - (lookahead_distance_ * lookahead_distance_);
            double discriminant = (b * b) - (4 * a * c);
            if (discriminant < 0)
            {
                // No intersection
                continue;
            }
            else
            {
                discriminant = sqrt(discriminant);
                t1 = (-b - discriminant) / (2 * a);
                t2 = (-b + discriminant) / (2 * a);
                if (t1 >= 0 && t1 <= 1)
                {
                    target_point_map_frame.header.frame_id = "map";
                    target_point_map_frame.point.x = ex + t1 * dx;
                    target_point_map_frame.point.y = ey + t1 * dy;
                    target_point_map_frame.point.z = 0.00;
                    tf2::doTransform(target_point_map_frame, target_point_car_frame, map_to_car_frame);
                    path_seq_ = i;
                    intersection_found = true;
                    break;
                }
                if (t2 >= 0 && t2 <= 1)
                {
                    target_point_map_frame.header.frame_id = "map";
                    target_point_map_frame.point.x = ex + t2 * dx;
                    target_point_map_frame.point.y = ey + t2 * dy;
                    target_point_map_frame.point.z = 0.00;
                    tf2::doTransform(target_point_map_frame, target_point_car_frame, map_to_car_frame);
                    path_seq_ = i;
                    intersection_found = true;
                    break;
                }
            }
        }
        if (intersection_found)
        {
            target_point_x = target_point_car_frame.point.x;
            target_point_y = target_point_car_frame.point.y;
            target_point_dist = sqrt(target_point_x * target_point_x + target_point_y * target_point_y);
            // RCLCPP_INFO(this->get_logger(), "Intersection with path found and distance from car is: %f", target_point_dist);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No intersection between lookahead radius circle and path found.");
            // Since no point of intersection found. Finding the closest point on the path and tracking that.
            double current_best_waypoint_dist = std::numeric_limits<double>::max();
            int current_best_waypoint_seq = path_seq_;
            for (int j = path_seq_; j < path_size_; j++)
            {
                double curr_x = path_to_track_map_frame_.poses[j].pose.position.x;
                double curr_y = path_to_track_map_frame_.poses[j].pose.position.y;
                double delta_x = curr_x - car_x;
                double delta_y = curr_y - car_y;
                double current_waypoint_dist = sqrt((delta_x * delta_x) + (delta_y * delta_y));
                if (current_waypoint_dist < current_best_waypoint_dist)
                {
                    current_best_waypoint_dist = current_waypoint_dist;
                    current_best_waypoint_seq = j;
                }
            }
            target_point_map_frame.header.frame_id = "map";
            target_point_map_frame.point.x = path_to_track_map_frame_.poses[current_best_waypoint_seq].pose.position.x;
            target_point_map_frame.point.y = path_to_track_map_frame_.poses[current_best_waypoint_seq].pose.position.y;
            target_point_map_frame.point.z = path_to_track_map_frame_.poses[current_best_waypoint_seq].pose.position.z;
            tf2::doTransform(target_point_map_frame, target_point_car_frame, map_to_car_frame);

            target_point_x = target_point_car_frame.point.x;
            target_point_y = target_point_car_frame.point.y;
            target_point_dist = sqrt(target_point_x * target_point_x + target_point_y * target_point_y);
            path_seq_ = current_best_waypoint_seq;
        }

        ackermann_msgs::msg::AckermannDriveStamped drive_command;

        if (goal_closeness(std::make_pair(car_x, car_y)) == 1)
        {
            stop_car();
            // Once goal reached, resetting class members so that next path can be followed correctly.
            path_seq_ = 0;
            path_detected_ = false;
            target_point_x = 0.0;
            target_point_y = 0.0;
            target_point_dist = 0.0;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "The lookahead distance used is: %f", target_point_dist);
            float steering_angle = 2 * (target_point_y) / (target_point_dist * target_point_dist);
            // RCLCPP_INFO(this->get_logger(), "Steering angle: %f degrees", steering_angle * (180 / PI));
            if (steering_angle > max_steering_angle_)
            {
                steering_angle = max_steering_angle_;
                drive_command.drive.speed = slowest_speed_;
            }
            else if (steering_angle < -max_steering_angle_)
            {
                steering_angle = -max_steering_angle_;
                drive_command.drive.speed = slowest_speed_;
            }
            else if (std::abs(steering_angle) >= 0.15)
            {
                drive_command.drive.speed = mid_speed_;
                if (goal_closeness(std::make_pair(car_x, car_y)) == 2)
                {
                    drive_command.drive.speed = slowest_speed_;
                }
            }
            else
            {
                drive_command.drive.speed = max_speed_;
                if (goal_closeness(std::make_pair(car_x, car_y)) == 2)
                {
                    drive_command.drive.speed = slowest_speed_;
                }
            }
            drive_command.header.stamp = this->now();
            drive_command.header.frame_id = "base_link";
            drive_command.drive.steering_angle = steering_angle;
            // RCLCPP_INFO(this->get_logger(), "The speed commanded is: %f", drive_command.drive.speed);
            drive_command_publisher_->publish(drive_command);
            // RCLCPP_INFO(this->get_logger(), "Steering angle commanded is: %f", steering_angle);
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No path detected to track.");
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    RCLCPP_INFO(node->get_logger(), "Pure Pursuit Controller node has been created!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
