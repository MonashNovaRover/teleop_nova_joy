/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <core/msg/drive_input_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include "teleop_nova_joy/teleop_nova_joy.hpp"

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

using namespace std::chrono_literals;

namespace
{
  constexpr auto DEFAULT_INPUT_TOPIC = "/joy";
  constexpr auto DEFAULT_OUTPUT_TOPIC = "/drive_input";
  constexpr auto DEFAULT_OUTPUT_TOPIC_TWIST = "/cmd_vel";

}

namespace teleop_nova_joy
{

enum Mode {
  STRAFE,
  PIVOT,
  DIFF
};

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopNovaJoy
 * directly into base nodes.
 */
struct TeleopNovaJoy::Impl
{
  TeleopNovaJoy& parent_;

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr, const std::string& which_map); rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<core::msg::DriveInputStamped>::SharedPtr drive_input_pub;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client;

  int64_t button_unlock;
  int64_t button_lock;

  //int64_t button_enable_turbo;

  int64_t button_strafe_mode;
  int64_t button_pivot_drive_mode;
  int64_t button_diff_drive_mode;

  int64_t button_autonomous_control;
  int64_t button_manual_control;

  std::map<std::string, int64_t> axis_linear_map;
  std::map<std::string, std::map<std::string, double>> scale_linear_map;

  std::map<std::string, int64_t> axis_angular_map;
  std::map<std::string, std::map<std::string, double>> scale_angular_map;

  bool sent_lock_msg;
  bool locked;
  bool manual_teleop;
  Mode mode; 
  Mode mode_old;
  
};

/**
 * Constructs TeleopNovaJoy.
 */
TeleopNovaJoy::TeleopNovaJoy(const rclcpp::NodeOptions& options) : Node("teleop_nova_joy_node", options)
{
  pimpl_ = new Impl{*this};

  pimpl_->drive_input_pub = this->create_publisher<core::msg::DriveInputStamped>(DEFAULT_OUTPUT_TOPIC, 50);
  pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(DEFAULT_OUTPUT_TOPIC_TWIST, 10);
  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(DEFAULT_INPUT_TOPIC, rclcpp::QoS(10),
    std::bind(&TeleopNovaJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

  //declaring default values for parameters
  pimpl_->button_unlock = this->declare_parameter("button_unlock", 11);
  pimpl_->button_lock = this->declare_parameter("button_lock", 15);
  pimpl_->button_manual_control = this->declare_parameter("button_manual_control", 1);
  pimpl_->button_autonomous_control = this->declare_parameter("button_autonomous_control", 0);
  pimpl_->button_strafe_mode = this->declare_parameter("button_strafe_mode", 6);
  pimpl_->button_pivot_drive_mode = this->declare_parameter("button_pivot_drive_mode", 7);
  pimpl_->button_diff_drive_mode = this->declare_parameter("button_diff_drive_mode", 4);

  pimpl_->switch_controller_client = this->create_client<controller_manager_msgs::srv::SwitchController>("switch_controller");

  if (!pimpl_->switch_controller_client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "SwitchController service not available.");
  }

  std::map<std::string, int64_t> default_linear_map{
    {"x", 5L},
    {"y", -1L},
    {"z", -1L},
  };
  this->declare_parameters("axis_linear", default_linear_map);
  this->get_parameters("axis_linear", pimpl_->axis_linear_map);

  RCLCPP_INFO(this->get_logger(), "axis_linear[x]: %ld", pimpl_->axis_linear_map["x"]);

  std::map<std::string, int64_t> default_angular_map{
    {"yaw", 2L},
    {"pitch", -1L},
    {"roll", -1L},
  };
  this->declare_parameters("axis_angular", default_angular_map);
  this->get_parameters("axis_angular", pimpl_->axis_angular_map);

  std::map<std::string, double> default_scale_linear_normal_map{
    {"x", 0.5},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear", default_scale_linear_normal_map);
  this->get_parameters("scale_linear", pimpl_->scale_linear_map["normal"]);

  std::map<std::string, double> default_scale_linear_turbo_map{
    {"x", 1.0},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear_turbo", default_scale_linear_turbo_map);
  this->get_parameters("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);

  std::map<std::string, double> default_scale_angular_normal_map{
    {"yaw", 0.5},
    {"pitch", 0.0},
    {"roll", 0.0},
  };
  this->declare_parameters("scale_angular", default_scale_angular_normal_map);
  this->get_parameters("scale_angular", pimpl_->scale_angular_map["normal"]);

  std::map<std::string, double> default_scale_angular_turbo_map{
    {"yaw", 1.0},
    {"pitch", 0.0},
    {"roll", 0.0},
  };
  this->declare_parameters("scale_angular_turbo", default_scale_angular_turbo_map);
  this->get_parameters("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);

  ROS_INFO_COND_NAMED(pimpl_->button_unlock, "TeleopNovaJoy",
      "Teleop unlock button %" PRId64 ".", pimpl_->button_unlock);
  ROS_INFO_COND_NAMED(pimpl_->button_unlock, "TeleopNovaJoy",
      "Teleop lock button %" PRId64 ".", pimpl_->button_lock);
  /*ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "TeleopNovaJoy",
    "Turbo on button %" PRId64 ".", pimpl_->turbo_button); */

  for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_linear_map.begin();
       it != pimpl_->axis_linear_map.end(); ++it)
  {
    ROS_INFO_COND_NAMED(it->second != -1L, "TeleopNovaJoy", "Linear axis %s on %" PRId64 " at scale %f.",
      it->first.c_str(), it->second, pimpl_->scale_linear_map["normal"][it->first]);
    /*ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopNovaJoy",
      "Turbo for linear axis %s is scale %f.", it->first.c_str(), pimpl_->scale_linear_map["turbo"][it->first]); */
  }

  for (std::map<std::string, int64_t>::iterator it = pimpl_->axis_angular_map.begin();
       it != pimpl_->axis_angular_map.end(); ++it)
  {
    ROS_INFO_COND_NAMED(it->second != -1L, "TeleopNovaJoy", "Angular axis %s on %" PRId64 " at scale %f.",
      it->first.c_str(), it->second, pimpl_->scale_angular_map["normal"][it->first]);
    /*ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0 && it->second != -1, "TeleopNovaJoy",
      "Turbo for angular axis %s is scale %f.", it->first.c_str(), pimpl_->scale_angular_map["turbo"][it->first]); */
  }

  pimpl_->sent_lock_msg = false;
  pimpl_->locked = false;
  pimpl_->mode = PIVOT;
  pimpl_->mode_old = PIVOT;
  pimpl_->manual_teleop = true;

  auto param_callback =
  [this](std::vector<rclcpp::Parameter> parameters)
  {
    static std::set<std::string> intparams = {"axis_linear.x", "axis_linear.y", "axis_linear.z",
                                              "axis_angular.yaw", "axis_angular.pitch", "axis_angular.roll",
                                              "button_unlock", "button_lock", "button_strafe_mode",
                                              "button_pivot_drive_mode", "button_diff_drive_mode",
                                              "button_autonomous_control", "button_manual_control"};
    static std::set<std::string> doubleparams = {"scale_linear.x", "scale_linear.y", "scale_linear.z",
                                                 //"scale_linear_turbo.x", "scale_linear_turbo.y", "scale_linear_turbo.z",
                                                 "scale_angular.yaw", "scale_angular.pitch", "scale_angular.roll"};
                                                 //"scale_angular_turbo.yaw", "scale_angular_turbo.pitch", "scale_angular_turbo.roll"};
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    // Loop to check if changed parameters are of expected data type
    for(const auto & parameter : parameters)
    {
      if (intparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
        {
          result.reason = "Only integer values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
      else if (doubleparams.count(parameter.get_name()) == 1)
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.reason = "Only double values can be set for '" + parameter.get_name() + "'.";
          RCLCPP_WARN(this->get_logger(), result.reason.c_str());
          result.successful = false;
          return result;
        }
      }
    }

    // Loop to assign changed parameters to the member variables
    for (const auto & parameter : parameters)
    {
      RCLCPP_INFO(this->get_logger(), "New parameter set: %s", parameter.get_name());
      if (parameter.get_name() == "button_unlock")
      {
        this->pimpl_->button_unlock = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "button_lock")
      {
        this->pimpl_->button_lock = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "button_strafe_mode")
      {
        this->pimpl_->button_strafe_mode = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "button_pivot_drive_mode")
      {
        this->pimpl_->button_pivot_drive_mode = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "button_diff_drive_mode")
      {
        this->pimpl_->button_diff_drive_mode = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "button_autonomous_control")
      {
        this->pimpl_->button_autonomous_control = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "button_manual_control")
      {
        this->pimpl_->button_manual_control = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_linear.x")
      {
        this->pimpl_->axis_linear_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_linear.y")
      {
        this->pimpl_->axis_linear_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_linear.z")
      {
        this->pimpl_->axis_linear_map["z"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_angular.yaw")
      {
        this->pimpl_->axis_angular_map["yaw"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_angular.pitch")
      {
        this->pimpl_->axis_angular_map["pitch"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_angular.roll")
      {
        this->pimpl_->axis_angular_map["roll"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      } else if (parameter.get_name() == "scale_linear.x") {
        this->pimpl_->scale_linear_map["normal"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_linear.y")
      {
        this->pimpl_->scale_linear_map["normal"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_linear.z")
      {
        this->pimpl_->scale_linear_map["normal"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      /*
      else if (parameter.get_name() == "scale_angular_turbo.yaw")
      {
        this->pimpl_->scale_angular_map["turbo"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular_turbo.pitch")
      {
        this->pimpl_->scale_angular_map["turbo"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular_turbo.roll")
      {
        this->pimpl_->scale_angular_map["turbo"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      */
      else if (parameter.get_name() == "scale_angular.yaw")
      {
        this->pimpl_->scale_angular_map["normal"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular.pitch")
      {
        this->pimpl_->scale_angular_map["normal"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_angular.roll")
      {
        this->pimpl_->scale_angular_map["normal"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
    }
    return result;
  };

  callback_handle = this->add_on_set_parameters_callback(param_callback);
}

TeleopNovaJoy::~TeleopNovaJoy()
{
  delete pimpl_;
}

double getVal(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int64_t>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      axis_map.at(fieldname) == -1L ||
      scale_map.find(fieldname) == scale_map.end() ||
      static_cast<int>(joy_msg->axes.size()) <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void TeleopNovaJoy::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                                         const std::string& which_map)
{
  //RCLCPP_INFO(parent_.get_logger(), "sending cmd_vel msg...");

  // Initializes with zeros by default.
  auto cmd_vel_msg = std::make_unique<core::msg::DriveInputStamped>();

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

  auto modeToController = [](int mode) -> std::string {
    switch (mode) {
      case STRAFE:
        return "strafe_controller";
      case PIVOT:
        return "pivot_drive_controller";
      case DIFF:
        return "nova_diff_drive_controller";
    }
  };

  RCLCPP_INFO(parent_.get_logger(), "Current mode: %s", modeToController(mode).c_str());

  if (mode != mode_old)
  {
    RCLCPP_INFO(parent_.get_logger(), "Changing from %s to %s", modeToController(mode_old).c_str(), modeToController(mode).c_str());
    request->activate_controllers.push_back(modeToController(mode));
    request->deactivate_controllers.push_back(modeToController(mode_old));
    /*
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    request->start_asap = true;
    builtin_interfaces::msg::Duration duration;
    duration.sec = 3;
    request->timeout = duration;
    */

    auto future = switch_controller_client->async_send_request(request);

    auto status = future.wait_for(3s);
    if (status == std::future_status::ready)
    {
      RCLCPP_INFO(parent_.get_logger(), "SwitchController service call successful.");
    }
    else
    {
      RCLCPP_ERROR(parent_.get_logger(), "SwitchController service call failed.");
    }
  }

  mode_old = mode;

  double angular = -getVal(joy_msg, axis_angular_map, scale_angular_map[which_map], "yaw");
  double linear = getVal(joy_msg, axis_linear_map, scale_linear_map[which_map], "x");

  RCLCPP_INFO(parent_.get_logger(), "angular: %f, linear: %f", angular, linear);

  cmd_vel_msg->drive_input.radius = angular == 0 ? INFINITY : (1.0 / pow(abs(angular), 2)) - 1;
  cmd_vel_msg->drive_input.direction = angular > 0 ? 1 : angular < 0 ? -1 : 0;
  cmd_vel_msg->drive_input.speed = linear;
  cmd_vel_msg->drive_input.mode = mode_old;

  //auto current_time = std::chrono::system_clock::now();
  //auto duration_since_epoch = current_time.time_since_epoch();
  //auto ros_time_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch);

  //cmd_vel_msg->header.stamp.sec = ros_time_stamp.count() / pow(10, 9);
  //cmd_vel_msg->header.stamp.nanosec = ros_time_stamp.count() % pow(10, 9);
  cmd_vel_msg->header.stamp = parent_.now();

  drive_input_pub->publish(std::move(cmd_vel_msg));

  sent_lock_msg = false;
}

void TeleopNovaJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  /*
  if (enable_turbo_button >= 0 &&
      static_cast<int>(joy_msg->buttons.size()) > enable_turbo_button &&
      joy_msg->buttons[enable_turbo_button])
  {
    sendCmdVelMsg(joy_msg, "turbo");
  }
  else if (!require_enable_button ||
	   (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
           joy_msg->buttons[enable_button]))
  {
    sendCmdVelMsg(joy_msg, "normal");
  }
  */

  if (joy_msg->buttons[button_unlock])
  {
    locked = false;
    RCLCPP_INFO(parent_.get_logger(), "BUTTON: unlock");
  } 
  else if (joy_msg->buttons[button_lock])
  {
    locked = true;
    RCLCPP_INFO(parent_.get_logger(), "BUTTON: lock");
  }
  
  if (joy_msg->buttons[button_autonomous_control])
  {
    manual_teleop = false; 
    RCLCPP_INFO(parent_.get_logger(), "BUTTON: autonomous_control");

  }
  else if (joy_msg->buttons[button_manual_control])
  {
    manual_teleop = true;
    RCLCPP_INFO(parent_.get_logger(), "BUTTON: manual_control");

  }

  if (joy_msg->buttons[button_strafe_mode])
  {
    mode = STRAFE;
    RCLCPP_INFO(parent_.get_logger(), "BUTTON: strafe_mode");

  }
  else if (joy_msg->buttons[button_diff_drive_mode])
  {
    mode = DIFF;
    RCLCPP_INFO(parent_.get_logger(), "BUTTON: diff_drive_mode");

  }
  else if (joy_msg->buttons[button_pivot_drive_mode])
  {
    mode = PIVOT;
    RCLCPP_INFO(parent_.get_logger(), "BUTTON: pivot_drive_mode");

  }

  if (!locked && manual_teleop)
  {
    sendCmdVelMsg(joy_msg, "normal");
  }
  else
  {
    // When lock button is pressed, immediately send a single no-motion command
    // in order to stop the robot.

    if (manual_teleop)
    {
      if (!sent_lock_msg)
      {
        // Initializes with zeros by default.
        auto cmd_vel_msg = std::make_unique<core::msg::DriveInputStamped>();
        drive_input_pub->publish(std::move(cmd_vel_msg));

        sent_lock_msg = true;
      }
    }
    else
    {
      //We want to continously publish a zero commmand to override any continuous autonomous messages
      
      // Initializes with zeros by default.
      auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
      cmd_vel_pub->publish(std::move(cmd_vel_msg));
    }
  }
}

}  // namespace teleop_nova_joy

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_nova_joy::TeleopNovaJoy)
