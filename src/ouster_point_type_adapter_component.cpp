#include "ouster_point_type_adapter/ouster_point_type_adapter_component.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "ouster_ros/include/ouster_ros/os_point.h"
#include "autoware_point_types/types.hpp"
#include <cmath>

namespace ouster_point_type_adapter
{
OusterPointTypeAdapter::OusterPointTypeAdapter(const rclcpp::NodeOptions & options)
: Node("ouster_point_type_adapter", options)
{
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", 10, std::bind(&OusterPointTypeAdapter::pointCloudCallback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);
}

// based on https://github.com/autowarefoundation/autoware.universe/issues/4978#issuecomment-1971777511
void OusterPointTypeAdapter::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Instantiate output messages
  auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

  // Instantiate pcl pointcloud message for the input point cloud
  pcl::PointCloud<ouster_ros::Point>::Ptr input_pointcloud(
    new pcl::PointCloud<ouster_ros::Point>);

  // Convert ros message to pcl
  pcl::fromROSMsg(*msg, *input_pointcloud);

  // Instantiate pcl pointcloud message for the output point cloud
  pcl::PointCloud<autoware_point_types::PointXYZIRCAEDT>::Ptr output_pointcloud(
    new pcl::PointCloud<autoware_point_types::PointXYZIRCAEDT>);
  
  output_pointcloud->reserve(input_pointcloud->points.size());

  // Convert pcl from ouster to pcl autoware format
  autoware_point_types::PointXYZIRCAEDT point{};
  for (const auto & p : input_pointcloud->points) {
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = uint8_t(p.reflectivity / 64);
    point.return_type = 0;
    point.channel = p.ring;
    point.azimuth = std::atan2(p.y, p.x);
    point.distance = float(p.range)/1000.0;
    point.elevation = std::asin(p.z / point.distance);
    point.time_stamp = p.t;
    output_pointcloud->points.emplace_back(point);
  }
  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = input_pointcloud->height;
  output_pointcloud->width  = input_pointcloud->width;

  // Convert pcl to ros message
  pcl::toROSMsg(*output_pointcloud, *pointcloud_msg);

  // Publish updated pointcloud message
  publisher_->publish(*pointcloud_msg);
}

}  // namespace ouster_point_type_adapter
RCLCPP_COMPONENTS_REGISTER_NODE(ouster_point_type_adapter::OusterPointTypeAdapter)
