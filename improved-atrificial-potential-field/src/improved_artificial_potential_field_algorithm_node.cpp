/* 
@file: improved_artificial_potential_field_algorithm_node.cpp
@author:

Description: Improved Artificial Potential Field (APF) Algorithm Node converted to ROS 2.
*/

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <algorithm>
#include <numeric>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <deque>
#include <chrono>
#include <limits>

// Constants
const float PI = 3.14159;

class APFController : public rclcpp::Node
{
private:
    // Publishers and Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr car_pose_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_command_publisher_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Member Variables
    nav_msgs::msg::Path path_to_track_map_frame_;
    int path_seq_;
    int path_size_;
    bool path_detected_;
    bool goal_reached_;
    std::pair<float, float> goal_point_;
    float max_steering_angle_ = 0.4189; // Max steering angle
    float mid_speed_;
    float slowest_speed_;
    float max_speed_;
    float scan_angle_min_;
    float scan_angle_max_;
    float scan_angle_increment_;
    float scan_range_min_;
    float scan_range_max_;
    double K_track_;
    double wheel_base_;

    // Tunable Parameters
    double obstacle_influence_distance_;
    double field_of_view_angle_degrees_;
    float lookahead_distance_;
    int sliding_window_length_;
    double R_avoid_factor_;
    double do_;
    float goal_threshold_distance_;
    std::string planner_topic_;

    // Velocity Profile Parameters
    double K_F_;
    double K_delta_;

public:
    // Constructor
    APFController();

private:
    // Helper Functions
    double Rad2Deg(double r) { return r * 180 / PI; }
    double Deg2Rad(double d) { return d * PI / 180; }
    std::vector<float> truncate_scan(const sensor_msgs::msg::LaserScan::SharedPtr& original_scan);
    void preprocessScan(std::vector<float>& truncated_scan_ranges);
    double angle_from_car_heading(int idx_of_range);
    std::vector<double> non_holonomic_dists(std::vector<float>& truncated_scan_ranges);
    std::pair<double, double> resultant_repulsive_force(std::vector<double>& non_holonomic_distances);
    double get_R_avoid(const std::pair<double, double>& force);
    int goal_closeness(const std::pair<float, float>& car_position);
    float dist(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2);
    void stop_car();
    void clip_angle(float& steering_angl);

    // Callback Functions
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void path_subscriber_callback(const nav_msgs::msg::Path::SharedPtr planner_path);
    void car_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr car_pose);
};

// Constructor Implementation
APFController::APFController()
    : Node("improved_artificial_potential_field_algorithm_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
    // Declare Parameters with Default Values
    this->declare_parameter<std::string>("planner_topic_", "/planned_path");
    this->declare_parameter<double>("obstacle_influence_distance_", 5.0);
    this->declare_parameter<double>("field_of_view_angle_degrees_", 180.0);
    this->declare_parameter<double>("lookahead_distance_", 2.0);
    this->declare_parameter<double>("R_avoid_factor_", 1.0);
    this->declare_parameter<double>("K_F_", 1.0);
    this->declare_parameter<double>("K_delta_", 1.0);
    this->declare_parameter<double>("wheel_base_", 0.3302);
    this->declare_parameter<double>("do_", 0.1);
    this->declare_parameter<int>("sliding_window_length_", 5);
    this->declare_parameter<double>("goal_threshold_distance_", 0.5);
    this->declare_parameter<double>("slowest_speed_", 1.0);
    this->declare_parameter<double>("mid_speed_", 2.5);
    this->declare_parameter<double>("max_speed_", 5.0);

    // Get Parameters
    planner_topic_ = this->get_parameter("planner_topic_").as_string();
    obstacle_influence_distance_ = this->get_parameter("obstacle_influence_distance_").as_double();
    field_of_view_angle_degrees_ = this->get_parameter("field_of_view_angle_degrees_").as_double();
    lookahead_distance_ = this->get_parameter("lookahead_distance_").as_double();
    R_avoid_factor_ = this->get_parameter("R_avoid_factor_").as_double();
    K_F_ = this->get_parameter("K_F_").as_double();
    K_delta_ = this->get_parameter("K_delta_").as_double();
    wheel_base_ = this->get_parameter("wheel_base_").as_double();
    do_ = this->get_parameter("do_").as_double();
    sliding_window_length_ = this->get_parameter("sliding_window_length_").as_int();
    goal_threshold_distance_ = this->get_parameter("goal_threshold_distance_").as_double();
    slowest_speed_ = this->get_parameter("slowest_speed_").as_double();
    mid_speed_ = this->get_parameter("mid_speed_").as_double();
    max_speed_ = this->get_parameter("max_speed_").as_double();

    // Initialize Publishers and Subscribers
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 1, std::bind(&APFController::scan_callback, this, std::placeholders::_1));
    drive_command_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", 1);
    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        planner_topic_, 1, std::bind(&APFController::path_subscriber_callback, this, std::placeholders::_1));
    car_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/gt_pose", 1, std::bind(&APFController::car_pose_callback, this, std::placeholders::_1));

    // Wait for TF buffer to fill
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Initialize Variables
    goal_reached_ = false;
    path_detected_ = false;
    path_seq_ = 0;
    K_track_ = 0.0;

    RCLCPP_INFO(this->get_logger(), "APFController node has been initialized");
}

// Implementations of Helper Functions
void APFController::clip_angle(float& steering_angle)
{
    if (steering_angle > max_steering_angle_)
    {
        steering_angle = max_steering_angle_;
    }
    else if (steering_angle < -max_steering_angle_)
    {
        steering_angle = -max_steering_angle_;
    }
}

void APFController::stop_car()
{
    ackermann_msgs::msg::AckermannDriveStamped drive_command;
    drive_command.header.stamp = this->now();
    drive_command.header.frame_id = "base_link";
    drive_command.drive.steering_angle = 0.0;
    drive_command.drive.speed = 0.0;
    drive_command_publisher_->publish(drive_command);
}

float APFController::dist(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2)
{
    float dx = pose2.pose.position.x - pose1.pose.position.x;
    float dy = pose2.pose.position.y - pose1.pose.position.y;
    return sqrt((dx * dx) + (dy * dy));
}

int APFController::goal_closeness(const std::pair<float, float>& car_position)
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

std::vector<float> APFController::truncate_scan(const sensor_msgs::msg::LaserScan::SharedPtr& original_scan)
{
    double half_fov = Deg2Rad(field_of_view_angle_degrees_ / 2);
    int fov_min_id = static_cast<int>((-half_fov - original_scan->angle_min) / original_scan->angle_increment);
    int fov_max_id = static_cast<int>((half_fov - original_scan->angle_min) / original_scan->angle_increment);
    return std::vector<float>(original_scan->ranges.begin() + fov_min_id, original_scan->ranges.begin() + fov_max_id);
}

void APFController::preprocessScan(std::vector<float>& truncated_scan_ranges)
{
    // Clip ranges between min and max
    for (float& r : truncated_scan_ranges)
    {
        if (r < scan_range_min_)
        {
            r = scan_range_min_;
        }
        else if (r > scan_range_max_)
        {
            r = scan_range_max_;
        }
    }
    // Apply sliding window running average
    std::vector<float> temp(truncated_scan_ranges.size(), 0.00);
    std::deque<float> running_avg;
    for (unsigned int i = 0; i < truncated_scan_ranges.size(); ++i)
    {
        running_avg.push_back(truncated_scan_ranges[i]);
        if (running_avg.size() > static_cast<size_t>(sliding_window_length_))
        {
            running_avg.pop_front();
        }
        float sum = std::accumulate(running_avg.begin(), running_avg.end(), 0.00f);
        temp[i] = sum / running_avg.size();
    }
    truncated_scan_ranges = temp;
}

double APFController::angle_from_car_heading(int idx_of_range)
{
    double half_fov = Deg2Rad(field_of_view_angle_degrees_ / 2);
    double angle = -half_fov + (idx_of_range) * (scan_angle_increment_);
    return angle;
}

std::vector<double> APFController::non_holonomic_dists(std::vector<float>& truncated_scan_ranges)
{
    std::vector<double> non_holonomic_distances(truncated_scan_ranges.size(), 0.00);
    for (size_t i = 0; i < truncated_scan_ranges.size(); i++)
    {
        double alpha_i = std::abs(angle_from_car_heading(static_cast<int>(i)));
        if (std::abs(alpha_i) == 0.0)
        {
            non_holonomic_distances[i] = truncated_scan_ranges[i];
        }
        else
        {
            if (truncated_scan_ranges[i] > lookahead_distance_)
            {
                non_holonomic_distances[i] = (lookahead_distance_ / (std::sin(alpha_i))) * alpha_i + (truncated_scan_ranges[i] - lookahead_distance_);
            }
            else
            {
                non_holonomic_distances[i] = (truncated_scan_ranges[i] / (std::sin(alpha_i))) * alpha_i;
            }
        }
    }
    return non_holonomic_distances;
}

std::pair<double, double> APFController::resultant_repulsive_force(std::vector<double>& non_holonomic_distances)
{
    double fx = 0.0;
    double fy = 0.0;
    for (size_t i = 0; i < non_holonomic_distances.size(); i++)
    {
        if (non_holonomic_distances[i] <= obstacle_influence_distance_)
        {
            double F = (1 / std::pow(obstacle_influence_distance_ + do_, 2)) - (1 / std::pow(non_holonomic_distances[i] + do_, 2));
            fx += std::cos(angle_from_car_heading(static_cast<int>(i))) * F;
            fy += std::sin(angle_from_car_heading(static_cast<int>(i))) * F;
        }
    }
    return std::make_pair(fx, fy);
}

double APFController::get_R_avoid(const std::pair<double, double>& force)
{
    double alpha_res = std::atan2(force.second, force.first);
    double mag_F = sqrt(std::pow(force.first, 2) + std::pow(force.second, 2));
    if (alpha_res == 0.0)
    {
        double R_avoid = 1 / (R_avoid_factor_ * mag_F);
        return std::min(R_avoid, 1000000.00);
    }
    else
    {
        int sign = std::abs(alpha_res) / alpha_res;
        double R_avoid = -1 * sign / (R_avoid_factor_ * mag_F);
        return std::min(R_avoid, 1000000.00);
    }
}

// Callback Functions Implementations
void APFController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    // Repulsive force and R_avoid calculation
    if (path_detected_)
    {
        scan_angle_min_ = scan_msg->angle_min;
        scan_angle_max_ = scan_msg->angle_max;
        scan_angle_increment_ = scan_msg->angle_increment;
        scan_range_min_ = scan_msg->range_min;
        scan_range_max_ = scan_msg->range_max;

        std::vector<float> truncated_scan_ranges = truncate_scan(scan_msg);
        preprocessScan(truncated_scan_ranges);

        std::vector<double> non_holo_dists = non_holonomic_dists(truncated_scan_ranges);
        std::pair<double, double> total_force = resultant_repulsive_force(non_holo_dists);
        double R_avoid = get_R_avoid(total_force);
        double K_avoid = 1 / R_avoid;
        double K_total = K_avoid + K_track_;

        // Publishing command
        ackermann_msgs::msg::AckermannDriveStamped apf_drive_command;
        apf_drive_command.header.stamp = this->now();
        apf_drive_command.header.frame_id = "base_link";
        float steer = wheel_base_ * K_total;
        clip_angle(steer);
        apf_drive_command.drive.steering_angle = steer;
        double F = sqrt(std::pow(total_force.first, 2) + std::pow(total_force.second, 2));
        double speed = std::max(2.5, (max_speed_ - K_F_ * F - K_delta_ * std::abs(steer)));
        RCLCPP_INFO(this->get_logger(), "Force: %f, speed: %f", F, speed);
        apf_drive_command.drive.speed = speed;
        drive_command_publisher_->publish(apf_drive_command);
    }
}

void APFController::path_subscriber_callback(const nav_msgs::msg::Path::SharedPtr planner_path)
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
        // Removed header.seq as it's not available in ROS 2
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

void APFController::car_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr car_pose)
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
        if (!tf_buffer_.canTransform("base_link", "map", tf2::TimePointZero))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for transform between 'base_link' and 'map'");
            return;
        }

        try
        {
            // Get the latest transform
            map_to_car_frame = tf_buffer_.lookupTransform("base_link", "map", tf2::TimePointZero);
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
        }
        else
        {
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

        if (goal_closeness(std::make_pair(car_x, car_y)) == 1)
        {
            stop_car();
            // Once goal reached resetting class members so that next path can be followed correctly.
            path_seq_ = 0;
            path_detected_ = false;
            goal_reached_ = true;
            target_point_x = 0.0;
            target_point_y = 0.0;
            target_point_dist = 0.0;
            K_track_ = 0.0;
        }
        else
        {
            float steering_angle = 2 * (target_point_y) / (target_point_dist * target_point_dist);
            clip_angle(steering_angle);
            K_track_ = steering_angle;
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No path detected to track.");
    }
}

// Main Function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<APFController>();
    RCLCPP_INFO(node->get_logger(), "APF node created!!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
