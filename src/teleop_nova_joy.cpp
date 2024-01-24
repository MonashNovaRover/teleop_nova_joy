/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, node_ list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, node_ list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from node_ software without specific prior written permission.

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

using std::placeholders::_1;
namespace teleop_nova_joy
{

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr TeleopNovaJoy::get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

  /**
   * Constructs TeleopNovaJoy.
   */
  TeleopNovaJoy::TeleopNovaJoy(const rclcpp::NodeOptions& options) : node_{ std::make_shared<rclcpp::Node>("teleop_nova_joy_node_", options)}
  {
    drive_input_pub = node_->create_publisher<core::msg::DriveInputStamped>(DEFAULT_OUTPUT_TOPIC, 50);
    cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>(DEFAULT_OUTPUT_TOPIC_TWIST, 10);
    
    joy_sub = node_->create_subscription<sensor_msgs::msg::Joy>(DEFAULT_INPUT_TOPIC, rclcpp::QoS(10), std::bind(&TeleopNovaJoy::joyCallback, this, _1));

    switch_controller_client = node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    mode = PIVOT;
    mode_old = PIVOT;
    
    param_listener_ = std::make_shared<ParamListener>(node_);
    params_ = param_listener_->get_params();

    if (param_listener_->is_old(params_)) {
      params_ = param_listener_->get_params();
    }
  }

  double TeleopNovaJoy::getVal(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::map<std::string, int64_t>& axis_map,
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

  void TeleopNovaJoy::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                                           const std::string& which_map)
  {
    //RCLCPP_INFO(node_->get_logger(), "sending cmd_vel msg...");

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

    //RCLCPP_INFO(node_->get_logger(), "Current mode: %s", modeToController(mode).c_str());

    if (mode != mode_old)
    {
      RCLCPP_INFO(node_->get_logger(), "Changing from %s to %s", modeToController(mode_old).c_str(), modeToController(mode).c_str());
      request->activate_controllers.push_back(modeToController(mode));
      request->deactivate_controllers.push_back(modeToController(mode_old));

      auto future = switch_controller_client->async_send_request(request);

      if (rclcpp::spin_until_future_complete(node_,future) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "service call failed :(");
      }

      //auto status = future.wait_for(3s);
      //if (status == std::future_status::ready)
      //{
      //  RCLCPP_INFO(node_->get_logger(), "SwitchController service call successful.");
      //}
      //else
      //{
      //  RCLCPP_ERROR(node_->get_logger(), "SwitchController service call failed.");
      //}
    }

    mode_old = mode;

    double angular = joy_msg->axes[params_.axis_angular.yaw] * params_.scale_angular.yaw;
    double linear = joy_msg->axes[params_.axis_linear.x] * params_.scale_linear.x;

    //RCLCPP_INFO(node_->get_logger(), "angular: %f, linear: %f", angular, linear);

    cmd_vel_msg->drive_input.radius = angular == 0 ? INFINITY : (1.0 / pow(abs(angular), 2)) - 1;
    cmd_vel_msg->drive_input.direction = angular > 0 ? 1 : angular < 0 ? -1 : 0;
    cmd_vel_msg->drive_input.speed = linear;
    cmd_vel_msg->drive_input.mode = mode_old;

    //auto current_time = std::chrono::system_clock::now();
    //auto duration_since_epoch = current_time.time_since_epoch();
    //auto ros_time_stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch);

    //cmd_vel_msg->header.stamp.sec = ros_time_stamp.count() / pow(10, 9);
    //cmd_vel_msg->header.stamp.nanosec = ros_time_stamp.count() % pow(10, 9);
    cmd_vel_msg->header.stamp = node_->now();

    drive_input_pub->publish(std::move(cmd_vel_msg));

    sent_lock_msg = false;
  }

  void TeleopNovaJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
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

    if (joy_msg->buttons[params_.button_unlock])
    {
      locked = false;
      RCLCPP_INFO(node_->get_logger(), "BUTTON: unlock");
    } 
    else if (joy_msg->buttons[params_.button_lock])
    {
      locked = true;
      RCLCPP_INFO(node_->get_logger(), "BUTTON: lock");
    }
    if (joy_msg->buttons[params_.button_autonomous_control])
    {
      manual_teleop = false; 
      RCLCPP_INFO(node_->get_logger(), "BUTTON: autonomous_control");
    }
    else if (joy_msg->buttons[params_.button_manual_control])
    {
      manual_teleop = true;
      RCLCPP_INFO(node_->get_logger(), "BUTTON: manual_control");
    }
    if (joy_msg->buttons[params_.button_strafe_mode])
    {
      mode = STRAFE;
      RCLCPP_INFO(node_->get_logger(), "BUTTON: strafe_mode");
    }
    else if (joy_msg->buttons[params_.button_diff_drive_mode])
    {
      mode = DIFF;
      RCLCPP_INFO(node_->get_logger(), "BUTTON: diff_drive_mode");
    }
    else if (joy_msg->buttons[params_.button_pivot_drive_mode])
    {
      mode = PIVOT;
      RCLCPP_INFO(node_->get_logger(), "BUTTON: pivot_drive_mode");
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(teleop_nova_joy::TeleopNovaJoy)
