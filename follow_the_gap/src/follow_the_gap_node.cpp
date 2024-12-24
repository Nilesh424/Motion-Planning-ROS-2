#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <algorithm>
#include <numeric>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <deque>

class FollowTheGap : public rclcpp::Node
{
public:
    FollowTheGap()
        : Node("follow_the_gap_node")
    {
        // Declare parameters with default values
        this->declare_parameter<float>("obstacle_distance_threshold_", 6.0);
        this->declare_parameter<float>("field_of_view_angle_degrees_", 180.0);
        this->declare_parameter<float>("safety_bubble_radius_", 0.5);
        this->declare_parameter<int>("sliding_window_length_", 5);
        this->declare_parameter<float>("safety_angle_", 20.0);
        this->declare_parameter<float>("buffer_angle_scaling_factor_", 6.0);

        // Get parameters
        obstacle_distance_threshold_ = this->get_parameter("obstacle_distance_threshold_").as_double();
        field_of_view_angle_degrees_ = this->get_parameter("field_of_view_angle_degrees_").as_double();
        safety_bubble_radius_ = this->get_parameter("safety_bubble_radius_").as_double();
        sliding_window_length_ = this->get_parameter("sliding_window_length_").as_int();
        safety_angle_ = this->get_parameter("safety_angle_").as_double();
        buffer_angle_scaling_factor_ = this->get_parameter("buffer_angle_scaling_factor_").as_double();

        // Subscriber
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1, std::bind(&FollowTheGap::scan_callback, this, std::placeholders::_1));

        // Publisher
        drive_command_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 1);

        RCLCPP_INFO(this->get_logger(), "Follow the Gap node initialized");
    }

private:
    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_command_publisher_;
    float max_steering_angle_ = 0.4189;

    // Parameters
    double obstacle_distance_threshold_;
    double field_of_view_angle_degrees_;
    double safety_bubble_radius_;
    int sliding_window_length_;
    double safety_angle_;
    double buffer_angle_scaling_factor_;

    const double PI = 3.14159;

    // Helper functions
    double Rad2Deg(double r) { return r * 180 / PI; }
    double Deg2Rad(double d) { return d * PI / 180; }

    std::vector<float> truncate_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
                                     float &field_of_view_min_angle,
                                     float &field_of_view_max_angle,
                                     float &scan_angle_increment)
    {
        double half_fov = Deg2Rad(field_of_view_angle_degrees_ / 2);
        int fov_min_id = static_cast<int>((-half_fov - scan_msg->angle_min) / scan_msg->angle_increment);
        int fov_max_id = static_cast<int>((half_fov - scan_msg->angle_min) / scan_msg->angle_increment);
        scan_angle_increment = scan_msg->angle_increment;
        field_of_view_min_angle = scan_msg->angle_min + fov_min_id * scan_msg->angle_increment;
        field_of_view_max_angle = scan_msg->angle_min + fov_max_id * scan_msg->angle_increment;
        return std::vector<float>(scan_msg->ranges.begin() + fov_min_id, scan_msg->ranges.begin() + fov_max_id);
    }

    void preprocessScan(std::vector<float> &truncated_scan_ranges, float range_min, float range_max)
    {
        // Clip ranges between min and max
        for (float &r : truncated_scan_ranges)
        {
            if (r < range_min)
                r = range_min;
            else if (r > range_max)
                r = range_max;
        }
        // Apply sliding window average
        std::vector<float> temp(truncated_scan_ranges.size(), 0.0f);
        std::deque<float> running_avg;
        for (size_t i = 0; i < truncated_scan_ranges.size(); ++i)
        {
            running_avg.push_back(truncated_scan_ranges[i]);
            if (running_avg.size() > static_cast<size_t>(sliding_window_length_))
            {
                running_avg.pop_front();
            }
            float sum = std::accumulate(running_avg.begin(), running_avg.end(), 0.0f);
            temp[i] = sum / running_avg.size();
        }
        truncated_scan_ranges = temp;
    }

    double findRadius(double sideA, double sideB, double included_angle)
    {
        // Using Law of Cosines
        return std::sqrt(sideA * sideA + sideB * sideB - 2 * sideA * sideB * std::cos(included_angle));
    }

    void create_angle_buffer(std::vector<float> &truncated_scan_ranges, int safety_angle_indices)
    {
        std::vector<std::pair<int, int>> obstacle_regions;
        int i = 0;
        while (i < static_cast<int>(truncated_scan_ranges.size()))
        {
            if (truncated_scan_ranges[i] == 0)
            {
                int obstacle_start_index = i;
                int j = i;
                while (j < static_cast<int>(truncated_scan_ranges.size()) && truncated_scan_ranges[j] == 0)
                {
                    ++j;
                }
                int obstacle_end_index = j;
                obstacle_regions.emplace_back(obstacle_start_index, obstacle_end_index);
                i = obstacle_end_index;
            }
            else
            {
                ++i;
            }
        }
        // Adding buffer around obstacles
        for (const auto &p : obstacle_regions)
        {
            for (int j = p.first; j >= std::max(0, p.first - safety_angle_indices); --j)
            {
                truncated_scan_ranges[j] = 0;
            }
            for (int j = p.second; j < std::min(static_cast<int>(truncated_scan_ranges.size()), p.second + safety_angle_indices); ++j)
            {
                truncated_scan_ranges[j] = 0;
            }
        }
    }

    size_t find_longest_free_space(const std::vector<float> &truncated_scan_ranges)
    {
        size_t count = 0, max_count = 0, max_end = 0;
        for (size_t i = 0; i <= truncated_scan_ranges.size(); ++i)
        {
            if (i < truncated_scan_ranges.size() && truncated_scan_ranges[i] > 0.0f)
            {
                ++count;
            }
            else
            {
                if (count > max_count)
                {
                    max_count = count;
                    max_end = i;
                }
                count = 0;
            }
        }
        auto max_range_ptr = std::max_element(truncated_scan_ranges.begin() + (max_end - max_count), truncated_scan_ranges.begin() + max_end);
        return std::distance(truncated_scan_ranges.begin(), max_range_ptr);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        float scan_angle_increment = 0.0f;
        float field_of_view_min_angle = 0.0f;
        float field_of_view_max_angle = 0.0f;
        std::vector<float> truncated_scan_ranges = truncate_scan(scan_msg, field_of_view_min_angle, field_of_view_max_angle, scan_angle_increment);

        preprocessScan(truncated_scan_ranges, scan_msg->range_min, scan_msg->range_max);

        // Find the closest obstacle
        auto min_ptr = std::min_element(truncated_scan_ranges.begin(), truncated_scan_ranges.end());
        auto min_range = *min_ptr;

        if (min_range < obstacle_distance_threshold_)
        {
            int min_idx = std::distance(truncated_scan_ranges.begin(), min_ptr);

            // Set ranges within safety bubble to zero
            for (size_t i = 0; i < truncated_scan_ranges.size(); ++i)
            {
                if (findRadius(truncated_scan_ranges[i], min_range, scan_angle_increment * std::abs(static_cast<int>(i) - min_idx)) <= safety_bubble_radius_)
                {
                    truncated_scan_ranges[i] = 0;
                }
            }

            float adaptive_buffer_angle = safety_angle_ + (buffer_angle_scaling_factor_ / min_range);
            int buffer_size_in_indices = static_cast<int>(Deg2Rad(adaptive_buffer_angle) / scan_msg->angle_increment);

            create_angle_buffer(truncated_scan_ranges, buffer_size_in_indices);

            size_t furthest_point_index = find_longest_free_space(truncated_scan_ranges);
            float steering_angle = field_of_view_min_angle + furthest_point_index * scan_angle_increment;

            // Clip steering angle
            steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);

            // Publish drive command
            ackermann_msgs::msg::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = this->now();
            drive_msg.header.frame_id = "laser";
            drive_msg.drive.steering_angle = steering_angle;
            drive_msg.drive.speed = 2.0;

            drive_command_publisher_->publish(drive_msg);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowTheGap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
