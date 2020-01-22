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

#include <abb_egm_hardware/abb_egm_hardware_sim.hpp>

namespace abb_egm_hardware
{
AbbEgmHardwareSim::AbbEgmHardwareSim(const std::string& name) : name_(name)
{
}

hardware_interface::hardware_interface_ret_t AbbEgmHardwareSim::init()
{
  node_ = rclcpp::Node::make_shared(name_);

  auto ret = hardware_interface::HW_RET_ERROR;

  for (std::size_t i = 0; i < n_joints_; ++i)
  {
    joint_state_handles_[i] = hardware_interface::JointStateHandle(joint_names_[i], &joint_position_[i],
                                                                   &joint_velocity_[i], &joint_effort_[i]);

    ret = register_joint_state_handle(&joint_state_handles_[i]);
    if (ret != hardware_interface::HW_RET_OK)
    {
      RCLCPP_WARN(node_->get_logger(), "Can't register joint state handle %s", joint_names_[i].c_str());
      return ret;
    }

    joint_command_handles_[i] = hardware_interface::JointCommandHandle(joint_names_[i], &joint_position_command_[i]);
    ret = register_joint_command_handle(&joint_command_handles_[i]);

    if (ret != hardware_interface::HW_RET_OK)
    {
      RCLCPP_WARN(node_->get_logger(), "Can't register joint command handle %s", joint_names_[i].c_str());
      return ret;
    }

    read_op_handles_[i] = hardware_interface::OperationModeHandle(
        read_op_handle_names_[i], reinterpret_cast<hardware_interface::OperationMode*>(&read_op_[i]));
    ret = register_operation_mode_handle(&read_op_handles_[i]);
    if (ret != hardware_interface::HW_RET_OK)
    {
      RCLCPP_WARN(node_->get_logger(), "Can't register read operation mode handle %s",
                  read_op_handle_names_[i].c_str());
      return ret;
    }

    write_op_handles_[i] = hardware_interface::OperationModeHandle(
        write_op_handle_names_[i], reinterpret_cast<hardware_interface::OperationMode*>(&write_op_[i]));
    ret = register_operation_mode_handle(&write_op_handles_[i]);
    if (ret != hardware_interface::HW_RET_OK)
    {
      RCLCPP_WARN(node_->get_logger(), "can't register write operation mode handle %s",
                  write_op_handle_names_[i].c_str());
      return ret;
    }
  }

  return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t AbbEgmHardwareSim::read()
{
  return hardware_interface::HW_RET_OK;
}

hardware_interface::hardware_interface_ret_t AbbEgmHardwareSim::write()
{
  for (size_t index = 0; index < n_joints_; ++index)
  {
    joint_position_[index] = joint_position_command_[index];
  }
  return hardware_interface::HW_RET_OK;
}

}  // namespace abb_egm_hardware
