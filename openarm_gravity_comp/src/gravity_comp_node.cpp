// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "openarm_gravity_comp/dynamics.hpp"

using openarm_gravity_comp::Dynamics;

namespace {

constexpr size_t ARM_DOF = 7;

// Joint names must match openarm_description URDF (v10.urdf.xacro with
// arm_prefix=right_/left_). Forward effort controllers in
// openarm_*_moveit_controllers.yaml are configured in the same order.
const std::vector<std::string> RIGHT_JOINT_NAMES = {
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7"};

const std::vector<std::string> LEFT_JOINT_NAMES = {
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7"};

// Conservative per-joint clamp [Nm]. Motors are Damiao
//   DM8009 (J1,J2): tMax = 54 Nm  -> clamp 30
//   DM4340 (J3,J4): tMax = 28 Nm  -> clamp 15
//   DM4310 (J5..J7): tMax = 10 Nm -> clamp 5/5/3
// Source: openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp.
const std::vector<double> TAU_LIMITS = {30.0, 30.0, 15.0, 15.0, 5.0, 5.0, 3.0};

// Gravity vector expressed in each arm's link0 frame, derived from the
// base-mount rpy in openarm_description/urdf/robot/v10.urdf.xacro (lines
// 48-52):
//   right_arm_base_rpy = (+pi/2, 0, 0) -> link0 gravity = (0, -9.81, 0)
//   left_arm_base_rpy  = (-pi/2, 0, 0) -> link0 gravity = (0, +9.81, 0)
//
// NOTE: openarm_hardware does NOT apply any direction multiplier on
// tau_commands_ (see openarm_simple_hardware.cpp:288-292), so the gravity
// vector here is the physical link0 gravity. Do not introduce an extra sign
// flip "for symmetry with openarmx" -- that codebase has a hardware-side -1
// that this codebase does not have.
constexpr double RIGHT_ARM_GY = -9.81;
constexpr double LEFT_ARM_GY = +9.81;

}  // namespace

class GravityCompNode : public rclcpp::Node {
 public:
  GravityCompNode() : Node("gravity_comp_node") {
    declare_parameter<std::string>("urdf_path", "");
    declare_parameter<double>("g_scale", 0.0);
    declare_parameter<bool>("enable_right", true);
    declare_parameter<bool>("enable_left", false);
    declare_parameter<bool>("enable_compensation", true);
    declare_parameter<bool>("verbose", false);

    g_scale_ = get_parameter("g_scale").as_double();
    enable_right_ = get_parameter("enable_right").as_bool();
    enable_left_ = get_parameter("enable_left").as_bool();
    enable_compensation_ = get_parameter("enable_compensation").as_bool();
    verbose_ = get_parameter("verbose").as_bool();

    const std::string urdf_path = get_parameter("urdf_path").as_string();
    if (urdf_path.empty()) {
      RCLCPP_FATAL(get_logger(),
                   "Parameter 'urdf_path' is required but was empty.");
      throw std::runtime_error("urdf_path not set");
    }

    if (enable_right_) {
      right_dyn_ = std::make_unique<Dynamics>(
          urdf_path, "openarm_right_link0", "openarm_right_link7");
      if (!right_dyn_->Init()) {
        throw std::runtime_error("Right arm KDL dynamics Init() failed");
      }
      right_dyn_->SetGravityVector(0.0, RIGHT_ARM_GY, 0.0);
      right_dyn_->PrintModelSummary();
      RCLCPP_INFO(get_logger(),
                  "Right arm dynamics ready: %zu joints, link0 gy=%.2f",
                  right_dyn_->NumJoints(), RIGHT_ARM_GY);
      right_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
          "/right_forward_effort_controller/commands", 10);
    }

    if (enable_left_) {
      left_dyn_ = std::make_unique<Dynamics>(
          urdf_path, "openarm_left_link0", "openarm_left_link7");
      if (!left_dyn_->Init()) {
        throw std::runtime_error("Left arm KDL dynamics Init() failed");
      }
      left_dyn_->SetGravityVector(0.0, LEFT_ARM_GY, 0.0);
      left_dyn_->PrintModelSummary();
      RCLCPP_INFO(get_logger(),
                  "Left arm dynamics ready: %zu joints, link0 gy=%.2f",
                  left_dyn_->NumJoints(), LEFT_ARM_GY);
      left_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
          "/left_forward_effort_controller/commands", 10);
    }

    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (const auto& p : params) {
            if (p.get_name() == "g_scale") {
              g_scale_ = p.as_double();
              RCLCPP_INFO(get_logger(), "g_scale updated to %.4f", g_scale_);
            } else if (p.get_name() == "enable_compensation") {
              enable_compensation_ = p.as_bool();
              RCLCPP_INFO(get_logger(), "enable_compensation = %s",
                          enable_compensation_ ? "true" : "false");
            } else if (p.get_name() == "verbose") {
              verbose_ = p.as_bool();
            }
          }
          return result;
        });

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&GravityCompNode::joint_state_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "gravity_comp_node started. g_scale=%.4f enable_right=%s enable_left=%s",
        g_scale_, enable_right_ ? "true" : "false",
        enable_left_ ? "true" : "false");
  }

 private:
  void joint_state_callback(
      const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (enable_right_ && right_dyn_) {
      publish_gravity_torques(msg, RIGHT_JOINT_NAMES, *right_dyn_, right_pub_,
                              "R");
    }
    if (enable_left_ && left_dyn_) {
      publish_gravity_torques(msg, LEFT_JOINT_NAMES, *left_dyn_, left_pub_,
                              "L");
    }
  }

  void publish_gravity_torques(
      const sensor_msgs::msg::JointState::SharedPtr& msg,
      const std::vector<std::string>& joint_names, Dynamics& dyn,
      const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr& pub,
      const char* arm_tag) {
    const size_t ndof = joint_names.size();

    std_msgs::msg::Float64MultiArray out;
    out.data.assign(ndof, 0.0);

    // When disabled, keep publishing zeros so the effort controller does not
    // hold stale feedforward from a previous cycle.
    if (!enable_compensation_) {
      pub->publish(out);
      return;
    }

    std::vector<double> q(ndof, 0.0);
    for (size_t j = 0; j < ndof; ++j) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[j]);
      if (it == msg->name.end()) {
        return;  // Wait until all arm joints appear in /joint_states.
      }
      const size_t idx = std::distance(msg->name.begin(), it);
      if (idx >= msg->position.size()) {
        return;
      }
      q[j] = msg->position[idx];
    }

    std::vector<double> tau_g(ndof, 0.0);
    dyn.GetGravity(q.data(), tau_g.data());

    bool any_bad = false;
    for (size_t j = 0; j < ndof; ++j) {
      double t = g_scale_ * tau_g[j];
      if (!std::isfinite(t)) {
        any_bad = true;
        t = 0.0;
      }
      const double limit = (j < TAU_LIMITS.size())
                               ? TAU_LIMITS[j]
                               : std::numeric_limits<double>::infinity();
      out.data[j] = std::clamp(t, -limit, limit);
    }

    if (any_bad) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "[%s] Non-finite gravity torque detected; clamped to 0",
                           arm_tag);
    }

    if (verbose_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[%s] tau=[%.2f %.2f %.2f %.2f %.2f %.2f %.2f] (g_scale=%.3f)",
          arm_tag, out.data[0], out.data[1], out.data[2], out.data[3],
          out.data[4], out.data[5], out.data[6], g_scale_);
    }

    pub->publish(out);
  }

  double g_scale_ = 0.0;
  bool enable_right_ = true;
  bool enable_left_ = false;
  bool enable_compensation_ = true;
  bool verbose_ = false;

  std::unique_ptr<Dynamics> right_dyn_;
  std::unique_ptr<Dynamics> left_dyn_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_cb_handle_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GravityCompNode>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("gravity_comp_node"), "Fatal: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
