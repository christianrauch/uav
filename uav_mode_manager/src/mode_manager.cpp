#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <control_msgs/msg/float64_values.hpp>
#include <control_msgs/msg/keys.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

class ModeManager : public rclcpp::Node
{
public:
  ModeManager(const rclcpp::NodeOptions & options) : Node("mode_manager", options)
  {
    mode_channel = this->declare_parameter<int8_t>("mode_channel_id");

    // ordered list of named modes
    mode_names = this->declare_parameter<std::vector<std::string>>("modes");

    if (mode_names.empty())
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "no modes defined");
    }

    for (const std::string & mode_name : mode_names)
    {
      const std::vector<std::string> controllers =
        this->declare_parameter<std::vector<std::string>>(
          "mode_controllers." + mode_name, std::vector<std::string>{});
      // superset of managed controllers
      managed_controllers.insert(controllers.cbegin(), controllers.cend());
      // mode subset
      mode_controllers[mode_name].insert(controllers.cbegin(), controllers.cend());
    }

    current_mode = {};

    sub_names = this->create_subscription<control_msgs::msg::Keys>(
      "~/names", rclcpp::QoS(10).transient_local(),
      std::bind(&ModeManager::on_channel_names, this, std::placeholders::_1));

    sub_values = this->create_subscription<control_msgs::msg::Float64Values>(
      "~/values", 10, std::bind(&ModeManager::on_channel_values, this, std::placeholders::_1));

    cli_switch = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    cli_list = this->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
  }

private:
  typedef controller_manager_msgs::srv::ListControllers::Response::SharedPtr
    ListControllersResponse;

  typedef controller_manager_msgs::srv::SwitchController::Response::SharedPtr
    SwitchControllerResponse;

  void on_channel_names(const control_msgs::msg::Keys::SharedPtr msg)
  {
    const std::vector<std::string> & channel_names = msg->keys;
    n_channel_names = channel_names.size();

    // find mode channel index
    const std::string mode_channel_name = "rc/channel/" + std::to_string(mode_channel);
    for (mode_channel_index = 0; (mode_channel_index < n_channel_names) &&
                                 (channel_names[mode_channel_index] != mode_channel_name);
         mode_channel_index++);

    if (mode_channel_index == n_channel_names)
    {
      // This is fatal since the topic is latched and the callback will only be called once. If we
      // cannot find the mode channel now, we will never know which value to look for.
      const std::string err =
        "Mode channel '" + mode_channel_name + "' not found in channel names!";
      RCLCPP_FATAL_STREAM(this->get_logger(), err);
      throw std::invalid_argument(err);
    }
  }

  void on_channel_values(const control_msgs::msg::Float64Values::SharedPtr msg)
  {
    if (!mtx_mode_switch.try_lock())
    {
      return;
    }

    if (n_channel_names == 0)
    {
      mtx_mode_switch.unlock();
      return;
    }

    if (msg->values.size() != n_channel_names)
    {
      RCLCPP_ERROR_STREAM(
        this->get_logger(), "Mismatch between channel names ("
                              << n_channel_names << ") and values (" << msg->values.size() << ")");
      mtx_mode_switch.unlock();
      return;
    }

    // calculate mode index based on channel value in range [0, 1]
    const double mode_channel_value = msg->values.at(mode_channel_index);
    const size_t n_modes = mode_names.size();
    const size_t new_mode_index = std::clamp<size_t>(mode_channel_value * n_modes, 0, n_modes - 1);
    next_mode = mode_names[new_mode_index];

    if (next_mode == current_mode)
    {
      mtx_mode_switch.unlock();
      return;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Switching to mode: " << next_mode);

    // check active controllers and switch to new mode
    cli_list->async_send_request(
      std::make_shared<controller_manager_msgs::srv::ListControllers::Request>(),
      std::bind(&ModeManager::cb_get_active_controllers_and_switch, this, std::placeholders::_1));
  }

  void cb_get_active_controllers_and_switch(
    const std::shared_future<ListControllersResponse> future)
  {
    const ListControllersResponse response = future.get();
    const std::vector<controller_manager_msgs::msg::ControllerState> & controllers =
      response->controller;

    managed_controllers_active.clear();
    managed_controllers_notactive.clear();
    for (const controller_manager_msgs::msg::ControllerState & controller : controllers)
    {
      if (managed_controllers.count(controller.name) == 0)
      {
        // not a managed controller
        continue;
      }

      if (controller.state == "active")
      {
        // managed controller that is currently active
        managed_controllers_active.insert(controller.name);
      }
      else
      {
        // managed controller that is currently not active (unconfigured, inactive, finalized)
        managed_controllers_notactive.insert(controller.name);
      }
    }

    switch_controllers();
  }

  void switch_controllers()
  {
    const std::set<std::string> & new_controllers = mode_controllers.at(next_mode);

    controller_manager_msgs::srv::SwitchController::Request::SharedPtr request =
      std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->activate_asap = true;

    // set of new controllers to activate, which are not yet active and should be activated
    std::set_intersection(
      //
      new_controllers.begin(), new_controllers.end(),
      //
      managed_controllers_notactive.begin(), managed_controllers_notactive.end(),
      //
      std::back_inserter(request->activate_controllers));

    // set of active controllers, which are not in requested mode and should be deactivated
    std::set_difference(
      //
      managed_controllers_active.begin(), managed_controllers_active.end(),
      //
      new_controllers.begin(), new_controllers.end(),
      //
      std::back_inserter(request->deactivate_controllers));

    if (!request->activate_controllers.empty())
    {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Activating controllers:");
      for (const std::string & controller : request->activate_controllers)
      {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "  " << controller);
      }
    }

    if (!request->deactivate_controllers.empty())
    {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Deactivating controllers:");
      for (const std::string & controller : request->deactivate_controllers)
      {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "  " << controller);
      }
    }

    cli_switch->async_send_request(
      request,
      [this](const std::shared_future<SwitchControllerResponse> future)
      {
        const SwitchControllerResponse response = future.get();
        if (response->ok)
        {
          RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Successfully switched from '" << current_mode << "' to '" << next_mode << "'");
          current_mode = next_mode;
        }
        else
        {
          RCLCPP_ERROR_STREAM(
            this->get_logger(), "Failed to switch from '" << current_mode << "' to '" << next_mode
                                                          << "': " << response->message);
        }
        mtx_mode_switch.unlock();
      });
  }

  rclcpp::Subscription<control_msgs::msg::Keys>::SharedPtr sub_names;
  rclcpp::Subscription<control_msgs::msg::Float64Values>::SharedPtr sub_values;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr cli_switch;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr cli_list;

  std::atomic<size_t> n_channel_names = 0;
  int8_t mode_channel;
  std::atomic<size_t> mode_channel_index;

  std::mutex mtx_mode_switch;
  std::string next_mode;

  std::set<std::string> managed_controllers;
  std::set<std::string> managed_controllers_active;
  std::set<std::string> managed_controllers_notactive;

  std::vector<std::string> mode_names;
  std::unordered_map<std::string, std::set<std::string>> mode_controllers;
  std::string current_mode = {};
};

RCLCPP_COMPONENTS_REGISTER_NODE(ModeManager)
