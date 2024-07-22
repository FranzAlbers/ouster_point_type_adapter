#ifndef ouster_point_type_adapter__ouster_point_type_adapter_COMPONENT_HPP_
#define ouster_point_type_adapter__ouster_point_type_adapter_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ouster_point_type_adapter
{
class OusterPointTypeAdapter : public rclcpp::Node
{
public:
  explicit OusterPointTypeAdapter(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  std::map<std::string, std::string> field_mappings_;
};
}  // namespace ouster_point_type_adapter

#endif  // ouster_point_type_adapter__ouster_point_type_adapter_COMPONENT_HPP_
