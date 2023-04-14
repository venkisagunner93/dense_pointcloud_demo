#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class SimplePointCloudSubscriber : public rclcpp::Node
{
public:
    SimplePointCloudSubscriber()
        : Node("simple_pointcloud_subscriber")
    {
        rclcpp::QoS qos(10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud", qos,
            std::bind(&SimplePointCloudSubscriber::pointcloud_callback, this, std::placeholders::_1));

        prev_time_ = this->now();
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const double dt = rclcpp::Duration(rclcpp::Time(msg->header.stamp) - prev_time_).seconds();
        prev_time_ = rclcpp::Time(msg->header.stamp);
        RCLCPP_INFO(this->get_logger(), "Time between two point clouds: %fs", dt);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Time prev_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePointCloudSubscriber>());
    rclcpp::shutdown();
    return 0;
}