#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{
double stampToSec(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<double>(stamp.sec) +
           static_cast<double>(stamp.nanosec) * 1e-9;
}

std::size_t pointCount(const sensor_msgs::msg::PointCloud2 &cloud)
{
    return static_cast<std::size_t>(cloud.width) *
           static_cast<std::size_t>(cloud.height);
}
}  // namespace

class LioSamRuntimeRecorder : public rclcpp::Node
{
public:
    LioSamRuntimeRecorder() : Node("liosam_runtime_recorder")
    {
        odom_topic_ = declare_parameter<std::string>(
            "odom_topic", "/lio_sam/mapping/odometry");
        incremental_odom_topic_ = declare_parameter<std::string>(
            "incremental_odom_topic", "/lio_sam/mapping/odometry_incremental");
        imu_odom_topic_ = declare_parameter<std::string>(
            "imu_odom_topic", "/odometry/imu_incremental");
        cloud_topic_ = declare_parameter<std::string>(
            "cloud_topic", "/lio_sam/mapping/cloud_registered");
        map_topic_ = declare_parameter<std::string>(
            "map_topic", "/lio_sam/mapping/map_global");
        deskew_topic_ = declare_parameter<std::string>(
            "deskew_topic", "/lio_sam/deskew/cloud_deskewed");
        corner_topic_ = declare_parameter<std::string>(
            "corner_topic", "/lio_sam/feature/cloud_corner");
        surface_topic_ = declare_parameter<std::string>(
            "surface_topic", "/lio_sam/feature/cloud_surface");
        csv_path_ = declare_parameter<std::string>(
            "csv_path",
            "/home/romi/Adaptive_FAST_LIO2/experiments/"
            "subt_mrs_hawkins_long_corridor/results/liosam_runtime_external.csv");
        csv_append_ = declare_parameter<bool>("csv_append", false);

        openCsv();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SensorDataQoS().keep_last(100),
            std::bind(
                &LioSamRuntimeRecorder::odomCallback, this,
                std::placeholders::_1));
        incremental_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            incremental_odom_topic_, rclcpp::SensorDataQoS().keep_last(100),
            std::bind(
                &LioSamRuntimeRecorder::incrementalOdomCallback, this,
                std::placeholders::_1));
        imu_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            imu_odom_topic_, rclcpp::SensorDataQoS().keep_last(2000),
            std::bind(
                &LioSamRuntimeRecorder::imuOdomCallback, this,
                std::placeholders::_1));

        cloud_sub_ = createCloudSubscription(
            cloud_topic_, &LioSamRuntimeRecorder::cloudCallback);
        map_sub_ = createCloudSubscription(
            map_topic_, &LioSamRuntimeRecorder::mapCallback);
        deskew_sub_ = createCloudSubscription(
            deskew_topic_, &LioSamRuntimeRecorder::deskewCallback);
        corner_sub_ = createCloudSubscription(
            corner_topic_, &LioSamRuntimeRecorder::cornerCallback);
        surface_sub_ = createCloudSubscription(
            surface_topic_, &LioSamRuntimeRecorder::surfaceCallback);

        RCLCPP_INFO(
            get_logger(),
            "Recording LIO-SAM outputs: odom=%s csv=%s",
            odom_topic_.c_str(), csv_path_.c_str());
    }

    ~LioSamRuntimeRecorder() override
    {
        if (csv_.is_open())
        {
            csv_.flush();
            csv_.close();
        }
    }

private:
    using CloudCallback =
        void (LioSamRuntimeRecorder::*)(
            const sensor_msgs::msg::PointCloud2::SharedPtr);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    createCloudSubscription(const std::string &topic, CloudCallback callback)
    {
        return create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, rclcpp::SensorDataQoS().keep_last(10),
            std::bind(callback, this, std::placeholders::_1));
    }

    void openCsv()
    {
        const std::filesystem::path path(csv_path_);
        const auto parent = path.parent_path();
        if (!parent.empty())
        {
            std::filesystem::create_directories(parent);
        }

        const std::ios_base::openmode mode =
            std::ios::out |
            (csv_append_ ? std::ios::app : std::ios::trunc);
        csv_.open(csv_path_, mode);
        if (!csv_.is_open())
        {
            throw std::runtime_error("failed to open CSV: " + csv_path_);
        }

        if (!csv_append_)
        {
            csv_
                << "frame,stamp,pos_x,pos_y,pos_z,"
                << "quat_x,quat_y,quat_z,quat_w,"
                << "cloud_points,cloud_stamp,map_points,map_stamp,"
                << "incremental_degenerate,incremental_stamp,"
                << "imu_speed,imu_stamp,"
                << "deskew_points,deskew_stamp,"
                << "corner_points,corner_stamp,"
                << "surface_points,surface_stamp"
                << std::endl;
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
             << latest_map_stamp_ << ","
             << (incremental_degenerate_ ? 1 : 0) << ","
             << incremental_stamp_ << ","
             << latest_imu_speed_ << ","
             << latest_imu_stamp_ << ","
             << latest_deskew_points_ << ","
             << latest_deskew_stamp_ << ","
             << latest_corner_points_ << ","
             << latest_corner_stamp_ << ","
             << latest_surface_points_ << ","
             << latest_surface_stamp_
             << std::endl;
    }

    void incrementalOdomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        incremental_degenerate_ =
            std::lround(msg->pose.covariance[0]) == 1;
        incremental_stamp_ = stampToSec(msg->header.stamp);
    }

    void imuOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &velocity = msg->twist.twist.linear;
        latest_imu_speed_ = std::sqrt(
            velocity.x * velocity.x +
            velocity.y * velocity.y +
            velocity.z * velocity.z);
        latest_imu_stamp_ = stampToSec(msg->header.stamp);
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

    void deskewCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_deskew_points_ = pointCount(*msg);
        latest_deskew_stamp_ = stampToSec(msg->header.stamp);
    }

    void cornerCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_corner_points_ = pointCount(*msg);
        latest_corner_stamp_ = stampToSec(msg->header.stamp);
    }

    void surfaceCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_surface_points_ = pointCount(*msg);
        latest_surface_stamp_ = stampToSec(msg->header.stamp);
    }

    std::string odom_topic_;
    std::string incremental_odom_topic_;
    std::string imu_odom_topic_;
    std::string cloud_topic_;
    std::string map_topic_;
    std::string deskew_topic_;
    std::string corner_topic_;
    std::string surface_topic_;
    std::string csv_path_;
    bool csv_append_ = false;

    std::ofstream csv_;
    std::uint64_t frame_ = 0;

    std::size_t latest_cloud_points_ = 0;
    std::size_t latest_map_points_ = 0;
    std::size_t latest_deskew_points_ = 0;
    std::size_t latest_corner_points_ = 0;
    std::size_t latest_surface_points_ = 0;
    double latest_cloud_stamp_ = 0.0;
    double latest_map_stamp_ = 0.0;
    double latest_deskew_stamp_ = 0.0;
    double latest_corner_stamp_ = 0.0;
    double latest_surface_stamp_ = 0.0;
    double latest_imu_speed_ = 0.0;
    double latest_imu_stamp_ = 0.0;
    double incremental_stamp_ = 0.0;
    bool incremental_degenerate_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        incremental_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr deskew_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr corner_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr surface_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LioSamRuntimeRecorder>());
    rclcpp::shutdown();
    return 0;
}
