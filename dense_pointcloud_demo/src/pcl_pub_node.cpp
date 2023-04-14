#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/qos.hpp>

using namespace std::chrono_literals;

class PointCloudPublisher : public rclcpp::Node
{
public:
    PointCloudPublisher()
        : Node("pointcloud_publisher"), count_(0)
    {
        rclcpp::QoS qos_profile(10); // Set the depth of the history cache to 100
        // rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", qos_profile);
        timer_ = this->create_wall_timer(100ms, std::bind(&PointCloudPublisher::publish_pointcloud, this));
        
        const unsigned int num_channels = 128;
        const unsigned int points_per_channel = 1024;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width = points_per_channel;
        cloud.height = num_channels;
        cloud.is_dense = true;
        cloud.points.resize(cloud.width * cloud.height);

        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        }

        pcl::toROSMsg(cloud, output_);

        prev_time_ = this->now();
    }

private:
    void publish_pointcloud()
    {
        sensor_msgs::msg::PointCloud2 output;
        output = output_;
        output.header.stamp = this->now();
        output.header.frame_id = "base_link";

        publisher_->publish(output);
        const double time_elapsed = rclcpp::Duration(this->now() - prev_time_).seconds();
        prev_time_ = this->now();
        // RCLCPP_INFO(this->get_logger(), "Time elapsed: %f", time_elapsed);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    size_t count_;
    sensor_msgs::msg::PointCloud2 output_;
    rclcpp::Time prev_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}