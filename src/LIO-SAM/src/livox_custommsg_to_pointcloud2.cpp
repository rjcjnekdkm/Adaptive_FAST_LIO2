#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

/**
 * @brief Convert Livox CustomMsg into the PointXYZIRT PointCloud2 format read by LIO-SAM.
 *
 * GEODE Gamma bags publish Livox CustomMsg on /livox/lidar, while LIO-SAM's
 * imageProjection node subscribes to sensor_msgs/PointCloud2.  This bridge
 * preserves each valid point's Livox line as `ring` and converts offset_time
 * from nanoseconds to seconds as the per-point `time` field required for IMU
 * deskewing.  No odometry or registration result is used as input.
 */
class LivoxCustomMsgToPointCloud2 : public rclcpp::Node
{
public:
    LivoxCustomMsgToPointCloud2()
        : Node("liosam_livox_adapter")
    {
        input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
        output_topic_ = declare_parameter<std::string>("output_topic", "/livox/points_lio_sam");
        scan_lines_ = declare_parameter<int>("scan_lines", 6);

        if (scan_lines_ <= 0 || scan_lines_ > 256)
        {
            throw std::runtime_error("scan_lines must be within [1, 256]");
        }

        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, rclcpp::SensorDataQoS());
        subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LivoxCustomMsgToPointCloud2::callback, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(), "Livox adapter: %s -> %s, scan_lines=%d",
            input_topic_.c_str(), output_topic_.c_str(), scan_lines_);
    }

private:
    static bool isValidReturn(const livox_ros_driver2::msg::CustomPoint &point)
    {
        const auto return_type = point.tag & 0x30;
        return return_type == 0x00 || return_type == 0x10;
    }

    void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr message)
    {
        std::vector<std::size_t> valid_indices;
        valid_indices.reserve(message->points.size());

        for (std::size_t index = 0; index < message->points.size(); ++index)
        {
            const auto &point = message->points[index];
            if (point.line >= scan_lines_ || !isValidReturn(point) ||
                !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                continue;
            }
            valid_indices.push_back(index);
        }

        sensor_msgs::msg::PointCloud2 output;
        output.header = message->header;
        sensor_msgs::PointCloud2Modifier modifier(output);
        modifier.setPointCloud2Fields(
            6,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "ring", 1, sensor_msgs::msg::PointField::UINT16,
            "time", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(valid_indices.size());

        sensor_msgs::PointCloud2Iterator<float> x_iterator(output, "x");
        sensor_msgs::PointCloud2Iterator<float> y_iterator(output, "y");
        sensor_msgs::PointCloud2Iterator<float> z_iterator(output, "z");
        sensor_msgs::PointCloud2Iterator<float> intensity_iterator(output, "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> ring_iterator(output, "ring");
        sensor_msgs::PointCloud2Iterator<float> time_iterator(output, "time");

        for (const auto index : valid_indices)
        {
            const auto &point = message->points[index];
            *x_iterator = point.x;
            *y_iterator = point.y;
            *z_iterator = point.z;
            *intensity_iterator = static_cast<float>(point.reflectivity);
            *ring_iterator = static_cast<uint16_t>(point.line);
            *time_iterator = static_cast<float>(point.offset_time) * 1e-9F;
            ++x_iterator;
            ++y_iterator;
            ++z_iterator;
            ++intensity_iterator;
            ++ring_iterator;
            ++time_iterator;
        }

        publisher_->publish(output);
    }

    std::string input_topic_;
    std::string output_topic_;
    int scan_lines_{};
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxCustomMsgToPointCloud2>());
    rclcpp::shutdown();
    return 0;
}
