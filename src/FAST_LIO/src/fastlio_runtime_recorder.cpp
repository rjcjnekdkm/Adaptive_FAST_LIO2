#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{
double stampToSec(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::size_t pointCount(const sensor_msgs::msg::PointCloud2 &cloud)
{
    return static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
}
}  // namespace

class FastLioRuntimeRecorder : public rclcpp::Node
{
public:
    FastLioRuntimeRecorder() : Node("fastlio_runtime_recorder")
    {
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
        cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered");
        map_topic_ = declare_parameter<std::string>("map_topic", "/Laser_map");
        csv_path_ = declare_parameter<std::string>(
            "csv_path",
            "/home/romi/Adaptive_FAST_LIO2/experiments/subt_mrs_hawkins_long_corridor/results/fastlio_runtime_external.csv");
        csv_append_ = declare_parameter<bool>("csv_append", false);

        openCsv();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 100,
            std::bind(&FastLioRuntimeRecorder::odomCallback, this, std::placeholders::_1));
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&FastLioRuntimeRecorder::cloudCallback, this, std::placeholders::_1));
        map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            map_topic_, rclcpp::SensorDataQoS(),
            std::bind(&FastLioRuntimeRecorder::mapCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Recording FAST_LIO outputs: odom=%s cloud=%s map=%s csv=%s",
            odom_topic_.c_str(), cloud_topic_.c_str(), map_topic_.c_str(), csv_path_.c_str());
    }

    ~FastLioRuntimeRecorder() override
    {
        if (csv_.is_open())
        {
            csv_.flush();
            csv_.close();
        }
    }

private:
    void openCsv()
    {
        const std::filesystem::path path(csv_path_);
        const auto parent = path.parent_path();
        if (!parent.empty())
        {
            std::filesystem::create_directories(parent);
        }

        const std::ios_base::openmode mode =
            std::ios::out | (csv_append_ ? std::ios::app : std::ios::trunc);
        csv_.open(csv_path_, mode);
        if (!csv_.is_open())
        {
            throw std::runtime_error("failed to open CSV: " + csv_path_);
        }

        if (!csv_append_)
        {
            csv_ << "frame,stamp,pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w,"
                 << "cloud_points,cloud_stamp,map_points,map_stamp" << std::endl;
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ++frame_;
        const auto &pose = msg->pose.pose;
        csv_ << std::fixed << std::setprecision(9)
             << frame_ << ","
             << stampToSec(msg->header.stamp) << ","
             << pose.position.x << ","
             << pose.position.y << ","
             << pose.position.z << ","
             << pose.orientation.x << ","
             << pose.orientation.y << ","
             << pose.orientation.z << ","
             << pose.orientation.w << ","
             << latest_cloud_points_ << ","
             << latest_cloud_stamp_ << ","
             << latest_map_points_ << ","
             << latest_map_stamp_
             << std::endl;
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_cloud_points_ = pointCount(*msg);
        latest_cloud_stamp_ = stampToSec(msg->header.stamp);
    }

    void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_map_points_ = pointCount(*msg);
        latest_map_stamp_ = stampToSec(msg->header.stamp);
    }

    std::string odom_topic_;
    std::string cloud_topic_;
    std::string map_topic_;
    std::string csv_path_;
    bool csv_append_ = false;

    std::ofstream csv_;
    std::uint64_t frame_ = 0;
    std::size_t latest_cloud_points_ = 0;
    std::size_t latest_map_points_ = 0;
    double latest_cloud_stamp_ = 0.0;
    double latest_map_stamp_ = 0.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastLioRuntimeRecorder>());
    rclcpp::shutdown();
    return 0;
}
