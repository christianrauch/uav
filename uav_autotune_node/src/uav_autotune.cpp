#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <uav_autotune/filter/AlphaFilter.hpp>

class UavAutotuneNode : public rclcpp::Node
{
public:
  explicit UavAutotuneNode(const rclcpp::NodeOptions & options)
  : Node("uav_autotune_node", options)
  {
    RCLCPP_INFO(get_logger(), "uav_autotune_node started");
  }

private:
  AlphaFilter<float> signal_filter_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(UavAutotuneNode)
