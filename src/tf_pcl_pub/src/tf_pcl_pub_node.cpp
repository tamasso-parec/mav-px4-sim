#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/transform_point_cloud.h>
#include <Eigen/Dense>

using namespace std::chrono_literals;

class PX4Visualizer : public rclcpp::Node
{
public:
    PX4Visualizer() : Node("visualizer")
    {
        // Configure subscriptions
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos_profile,
            std::bind(&PX4Visualizer::vehicle_attitude_callback, this, std::placeholders::_1));

        local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos_profile,
            std::bind(&PX4Visualizer::vehicle_local_position_callback, this, std::placeholders::_1));

        setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos_profile,
            std::bind(&PX4Visualizer::trajectory_setpoint_callback, this, std::placeholders::_1));

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/x500_depth/point_cloud", qos_profile,
            std::bind(&PX4Visualizer::pointcloud_callback, this, std::placeholders::_1));

        // vehicle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        //     "/px4_visualizer/vehicle_pose", 10);

        // vehicle_vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        //     "/px4_visualizer/vehicle_velocity", 10);

        // vehicle_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        //     "/px4_visualizer/vehicle_path", 10);

        // setpoint_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        //     "/px4_visualizer/setpoint_path", 10);

        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/px4_visualizer/pointcloud", 5);

        vehicle_attitude_ = Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0);
        vehicle_local_position_ = Eigen::Vector3f(0.0, 0.0, 0.0);
        vehicle_local_velocity_ = Eigen::Vector3f(0.0, 0.0, 0.0);
        setpoint_position_ = Eigen::Vector3f(0.0, 0.0, 0.0);

        trail_size_ = 1000;
        last_local_pos_update_ = 0.0;
        path_clearing_timeout_ = this->declare_parameter("path_clearing_timeout", -1.0);

        timer_ = this->create_wall_timer(
            50ms, std::bind(&PX4Visualizer::cmdloop_callback, this));
    }

private:
    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        vehicle_attitude_ = Eigen::Quaternionf(msg->q[0], msg->q[1], -msg->q[2], -msg->q[3]);
    }

    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        if (path_clearing_timeout_ >= 0 &&
            ((this->now().seconds() - last_local_pos_update_) > path_clearing_timeout_))
        {
            vehicle_path_msg_.poses.clear();
        }
        last_local_pos_update_ = this->now().seconds();

        vehicle_local_position_ = Eigen::Vector3f(msg->x, -msg->y, -msg->z);
        vehicle_local_velocity_ = Eigen::Vector3f(msg->vx, -msg->vy, -msg->vz);
    }

    void trajectory_setpoint_callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
    {
        setpoint_position_ = Eigen::Vector3f(msg->position[0], -msg->position[1], -msg->position[2]);
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        Eigen::Matrix3f rotation_matrix = vehicle_attitude_.toRotationMatrix();
        Eigen::Vector3f translation = vehicle_local_position_;

        Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
        transform_matrix.topLeftCorner(3,3) = rotation_matrix;
        transform_matrix.topRightCorner(3,1) = translation;

        pcl::transformPointCloud(cloud, cloud, transform_matrix);

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "map";
        pointcloud_pub_->publish(output);
    }

    void cmdloop_callback()
    {
        // auto vehicle_pose_msg = vector2PoseMsg("map", vehicle_local_position_, vehicle_attitude_);
        // vehicle_pose_pub_->publish(vehicle_pose_msg);

        // vehicle_path_msg_.header = vehicle_pose_msg.header;
        // append_vehicle_path(vehicle_pose_msg);
        // vehicle_path_pub_->publish(vehicle_path_msg_);

        // auto setpoint_pose_msg = vector2PoseMsg("map", setpoint_position_, vehicle_attitude_);
        // setpoint_path_msg_.header = setpoint_pose_msg.header;
        // append_setpoint_path(setpoint_pose_msg);
        // setpoint_path_pub_->publish(setpoint_path_msg_);

        // auto velocity_msg = create_arrow_marker(1, vehicle_local_position_, vehicle_local_velocity_);
        // vehicle_vel_pub_->publish(velocity_msg);
    }

    geometry_msgs::msg::PoseStamped vector2PoseMsg(const std::string &frame_id, const Eigen::Vector3f &position, const Eigen::Quaternionf &attitude)
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = frame_id;
        pose_msg.pose.orientation.w = attitude.w();
        pose_msg.pose.orientation.x = attitude.x();
        pose_msg.pose.orientation.y = attitude.y();
        pose_msg.pose.orientation.z = attitude.z();
        pose_msg.pose.position.x = position.x();
        pose_msg.pose.position.y = position.y();
        pose_msg.pose.position.z = position.z();
        return pose_msg;
    }

    visualization_msgs::msg::Marker create_arrow_marker(int id, const Eigen::Vector3f &tail, const Eigen::Vector3f &vector)
    {
        visualization_msgs::msg::Marker msg;
        msg.action = visualization_msgs::msg::Marker::ADD;
        msg.header.frame_id = "map";
        msg.ns = "arrow";
        msg.id = id;
        msg.type = visualization_msgs::msg::Marker::ARROW;
        msg.scale.x = 0.1;
        msg.scale.y = 0.2;
        msg.scale.z = 0.0;
        msg.color.r = 0.5;
        msg.color.g = 0.5;
        msg.color.b = 0.0;
        msg.color.a = 1.0;
        geometry_msgs::msg::Point tail_point, head_point;
        tail_point.x = tail.x();
        tail_point.y = tail.y();
        tail_point.z = tail.z();
        head_point.x = tail.x() + 0.3 * vector.x();
        head_point.y = tail.y() + 0.3 * vector.y();
        head_point.z = tail.z() + 0.3 * vector.z();
        msg.points.push_back(tail_point);
        msg.points.push_back(head_point);
        return msg;
    }

    void append_vehicle_path(const geometry_msgs::msg::PoseStamped &msg)
    {
        vehicle_path_msg_.poses.push_back(msg);
        if (vehicle_path_msg_.poses.size() > trail_size_)
        {
            vehicle_path_msg_.poses.erase(vehicle_path_msg_.poses.begin());
        }
    }

    void append_setpoint_path(const geometry_msgs::msg::PoseStamped &msg)
    {
        setpoint_path_msg_.poses.push_back(msg);
        if (setpoint_path_msg_.poses.size() > trail_size_)
        {
            setpoint_path_msg_.poses.erase(setpoint_path_msg_.poses.begin());
        }
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
    // rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_vel_pub_;
    // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
    // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr setpoint_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    Eigen::Quaternionf vehicle_attitude_;
    Eigen::Vector3f vehicle_local_position_;
    Eigen::Vector3f vehicle_local_velocity_;
    Eigen::Vector3f setpoint_position_;

    nav_msgs::msg::Path vehicle_path_msg_;
    nav_msgs::msg::Path setpoint_path_msg_;

    double last_local_pos_update_;
    double path_clearing_timeout_;
    size_t trail_size_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PX4Visualizer>());
    rclcpp::shutdown();
    return 0;
}