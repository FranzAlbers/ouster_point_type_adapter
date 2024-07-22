#include "rclcpp/rclcpp.hpp"
#include "ouster_point_type_adapter/ouster_point_type_adapter_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ouster_point_type_adapter::OusterPointTypeAdapter>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}