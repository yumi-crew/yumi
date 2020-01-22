// Copyright 2019 Norwegian University of Science and Technology.
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

#ifndef ABB_EGM_HARDWARE__ABB_EGM_HARDWARE_HPP_
#define ABB_EGM_HARDWARE__ABB_EGM_HARDWARE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>

#include <angles/angles.h>

#include <hardware_interface/robot_hardware.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <abb_egm_hardware/visibility_control.h>

namespace abb_egm_hardware
{
class AbbEgmHardwareSim : public hardware_interface::RobotHardware
{
public:
  AbbEgmHardwareSim(const std::string& name);

  ABB_EGM_HARDWARE_PUBLIC
  hardware_interface::hardware_interface_ret_t init();

  ABB_EGM_HARDWARE_PUBLIC
  hardware_interface::hardware_interface_ret_t read();

  ABB_EGM_HARDWARE_PUBLIC
  hardware_interface::hardware_interface_ret_t write();

private:
  std::string name_;
  std::shared_ptr<rclcpp::Node> node_;

  static constexpr unsigned short n_joints_ = 14;

  std::array<hardware_interface::JointStateHandle, n_joints_> joint_state_handles_;
  std::array<hardware_interface::JointCommandHandle, n_joints_> joint_command_handles_;
  std::array<hardware_interface::OperationModeHandle, n_joints_> read_op_handles_;
  std::array<hardware_interface::OperationModeHandle, n_joints_> write_op_handles_;

  std::array<std::string, n_joints_> joint_names_ = {
    "yumi_joint_1_l", "yumi_joint_2_l", "yumi_joint_7_l", "yumi_joint_3_l", "yumi_joint_4_l",
    "yumi_joint_5_l", "yumi_joint_6_l", "yumi_joint_1_r", "yumi_joint_2_r", "yumi_joint_7_r",
    "yumi_joint_3_r", "yumi_joint_4_r", "yumi_joint_5_r", "yumi_joint_6_r",
  };

  std::array<double, n_joints_> joint_position_{};
  std::array<double, n_joints_> joint_velocity_{};
  std::array<double, n_joints_> joint_effort_{};
  std::array<double, n_joints_> joint_position_command_{};

  std::array<bool, n_joints_> read_op_ = { false, false, false, false, false, false, false,
                                           false, false, false, false, false, false, false };
  std::array<bool, n_joints_> write_op_ = { true, true, true, true, true, true, true,
                                            true, true, true, true, true, true, true };

  std::array<std::string, n_joints_> read_op_handle_names_ = { "read1_l", "read2_l", "read7_l", "read3_l", "read4_l",
                                                               "read5_l", "read6_l", "read1_r", "read2_r", "read7_r",
                                                               "read3_r", "read4_r", "read5_r", "read6_r" };

  std::array<std::string, n_joints_> write_op_handle_names_ = { "write1_l", "write2_l", "write7_l", "write3_l",
                                                                "write4_l", "write5_l", "write6_l", "write1_r",
                                                                "write2_r", "write7_r", "write3_r", "write4_r",
                                                                "write5_r", "write6_r" };
};

}  // namespace abb_egm_hardware

#endif  // ABB_EGM_HARDWARE__ABB_EGM_HARDWARE_HPP_