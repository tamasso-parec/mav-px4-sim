#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PCDPublisher : public rclcpp::Node
{
public:
    PCDPublisher()
    : Node("pcd_publisher")
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("pcd_file", "pointcloud.pcd");
        this->declare_parameter<std::string>("topic_name", "pointcloud_topic");

        // Get parameter values
        this->get_parameter("pcd_file", pcd_file_);
        this->get_parameter("topic_name", topic_name_);

        // Initialize publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name_, 10);

        // Load and publish the point cloud
        publish_point_cloud();
    }

private:
    void publish_point_cloud()
    {
        // Load PCD file
        pcl::PointCloud<pcl::PointXYZ> cloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", pcd_file_.c_str());
            return;
        }

        // Convert to ROS 2 message
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "map";

        // Publish the point cloud
        publisher_->publish(output);
        RCLCPP_INFO(this->get_logger(), "Published point cloud from %s on topic %s", pcd_file_.c_str(), topic_name_.c_str());
    }

    std::string pcd_file_;
    std::string topic_name_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// ros2 run nome_pkg nome_exe --ros-args -p pcd_file:=/path/to/pointcloud.pcd -p topic_name:=/pointcloud_topic