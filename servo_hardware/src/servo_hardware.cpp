// Copyright 2026 Gerardo Puga
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

#include "servo_hardware/servo_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "servo_hardware/serial_port.hpp"
#include "servo_protocol_decoder/packets.hpp"
#include "servo_protocol_decoder/servo_protocol_decoder.hpp"

namespace servo_hardware
{
hardware_interface::CallbackReturn ServoHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read hardware parameters
  auto it_device = info_.hardware_parameters.find("device");
  if (it_device == info_.hardware_parameters.end()) {
    RCLCPP_FATAL(get_logger(),
                 "Parameter 'device' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }
  device_ = it_device->second;

  auto it_prefix = info_.hardware_parameters.find("prefix");
  if (it_prefix != info_.hardware_parameters.end()) {
    prefix_ = it_prefix->second;
  } else {
    prefix_.clear();
  }

  auto it_baudrate = info_.hardware_parameters.find("baudrate");
  if (it_baudrate != info_.hardware_parameters.end()) {
    baudrate_ = std::stoul(it_baudrate->second);
  } else {
    baudrate_ = 115200;  // Default baudrate
  }

  RCLCPP_INFO(get_logger(), "Configured device: %s", device_.c_str());
  RCLCPP_INFO(get_logger(), "Configured prefix: %s", prefix_.c_str());
  RCLCPP_INFO(get_logger(), "Configured baudrate: %u", baudrate_);

  // Initialize list of supported interfaces
  populate_state_definitions();
  populate_command_definitions();

  // Validate all joints
  for (const auto & component : info_.joints) {
    // Validate and mark requested state interfaces
    if ((validate_and_mark_requested_state_interfaces(component) !=
      hardware_interface::CallbackReturn::SUCCESS) ||
      (validate_and_mark_requested_command_interfaces(component) !=
      hardware_interface::CallbackReturn::SUCCESS))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Initialize state storage
  servo_0_state_ = 0.0;
  servo_1_state_ = 0.0;
  servo_2_state_ = 0.0;
  servo_3_state_ = 0.0;

  // Initialize command storage
  servo_0_command_ = 0.0;
  servo_1_command_ = 0.0;
  servo_2_command_ = 0.0;
  servo_3_command_ = 0.0;

  // Initialize realtime data structures
  status_box_ = std::make_unique<realtime_tools::RealtimeThreadSafeBox<
        servo_protocol_decoder::ServoStatusPayload>>(
      servo_protocol_decoder::ServoStatusPayload{});
  command_queue_ = std::make_unique<realtime_tools::LockFreeSPSCQueue<
        servo_protocol_decoder::ServoCommandPayload>>(10);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring servo hardware interface...");

  // Log which interfaces are active
  RCLCPP_INFO(get_logger(), "Active state interfaces:");
  for (const auto & [name, data] : state_interfaces_) {
    if (data.state.requested) {
      RCLCPP_INFO(get_logger(), "  - %s", name.c_str());
    }
  }
  RCLCPP_INFO(get_logger(), "Active command interfaces:");
  for (const auto & [name, data] : command_interfaces_) {
    if (data.state.requested) {
      RCLCPP_INFO(get_logger(), "  - %s", name.c_str());
    }
  }

  // Reset values when configuring hardware (only for requested interfaces)
  for (const auto & [name, data] : state_interfaces_) {
    if (data.state.requested) {
      set_state(name, 0.0);
    }
  }
  for (const auto & [name, data] : command_interfaces_) {
    if (data.state.requested) {
      set_command(name, 0.0);
    }
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating servo hardware interface...");

  try {
    // Configure serial port
    servo_hardware::SerialPort::PortConfiguration config;
    config.baud_rate =
      static_cast<servo_hardware::SerialPort::BaudRate>(baudrate_);
    config.data_bits = servo_hardware::SerialPort::DataBits::EIGHT;
    config.stop_bits = servo_hardware::SerialPort::StopBits::ONE;
    config.parity = servo_hardware::SerialPort::Parity::NONE;
    config.flow_control = servo_hardware::SerialPort::FlowControl::NONE;

    // Create and open serial port
    serial_port_ = std::make_unique<servo_hardware::SerialPort>();
    if (!serial_port_->open(device_, config)) {
      throw std::runtime_error("Failed to open serial port");
    }
    RCLCPP_INFO(get_logger(), "Opened serial port on %s", device_.c_str());

    // Create protocol decoder
    protocol_decoder_ =
      std::make_unique<servo_protocol_decoder::ServoProtocolDecoder>();

    // Set up callback for status packets
    protocol_decoder_->setServoStatusCallback(
      [this](const servo_protocol_decoder::ServoStatusPayload & packet) {
        status_box_->set(packet);
        state_message_received_.store(true);
        });

    // Set up receive callback
    serial_port_->setCallback([this](const uint8_t * data, std::size_t size) {
        if (size > 0 && protocol_decoder_) {
          protocol_decoder_->processStream(data, size);
        }
    });
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize serial communication: %s",
                 e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Synchronize command with current state
  for (const auto & [name, data] : state_interfaces_) {
    if (data.state.requested) {
      set_command(name, get_state(name));
    }
  }

  // Start background thread
  halting_ = false;
  background_thread_ =
    std::thread(&ServoHardware::background_thread_function, this);

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ServoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating servo hardware interface...");

  // Stop background thread
  if (background_thread_.joinable()) {
    halting_ = true;
    cv_.notify_all();
    background_thread_.join();
    RCLCPP_INFO(get_logger(), "Stopped background thread");
  }

  // Close serial port and cleanup
  if (serial_port_) {
    serial_port_->close();
    serial_port_.reset();
    RCLCPP_INFO(get_logger(), "Closed serial port");
  }

  protocol_decoder_.reset();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ServoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Get latest status from RealtimeThreadSafeBox and update state variables
  servo_protocol_decoder::ServoStatusPayload status;
  status_box_->get(status);

  if (!state_message_received_.load()) {
    RCLCPP_WARN(get_logger(), "No state message packet yet");
  } else {
    servo_0_state_ = status.servo[0];
    servo_1_state_ = status.servo[1];
    servo_2_state_ = status.servo[2];
    servo_3_state_ = status.servo[3];
  }

  // Update exported state interfaces from atomic variables
  for (const auto & [name, descr] : joint_state_interfaces_) {
    auto it = state_interfaces_.find(name);
    if (it != state_interfaces_.end() && it->second.state.requested &&
      it->second.get_value)
    {
      set_state(name, it->second.get_value());
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ServoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool has_valid_command = false;
  for (const auto & [name, descr] : joint_command_interfaces_) {
    auto it = command_interfaces_.find(name);
    if (it != command_interfaces_.end() && it->second.state.requested &&
      it->second.set_command)
    {
      const double command_value = get_command(name);

      if (it->second.state.claimed && !std::isnan(command_value)) {
        if (it->second.state.post_claim_delay > 0) {
          RCLCPP_WARN(get_logger(),
                      "Ignoring command for interface %s due to post-claim delay",
                      name.c_str());
          it->second.state.post_claim_delay--;
          continue;
        }

        it->second.set_command(command_value);
        has_valid_command = true;
      }
    }
  }

  servo_protocol_decoder::ServoCommandPayload command_packet;
  command_packet.servo[0] = servo_0_command_;
  command_packet.servo[1] = servo_1_command_;
  command_packet.servo[2] = servo_2_command_;
  command_packet.servo[3] = servo_3_command_;

  // Queue the command packet (lock-free) and wake up background thread
  // Only push if at least one command is valid (not NaN)
  if (has_valid_command) {
    if (command_queue_->push(command_packet)) {
      cv_.notify_one();
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ServoHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  RCLCPP_INFO(get_logger(), "Performing command mode switch");

  // Release (unclaim) stop interfaces
  for (const auto & interface_name : stop_interfaces) {
    auto it = command_interfaces_.find(interface_name);
    if (it != command_interfaces_.end()) {
      it->second.state.claimed = false;
      RCLCPP_INFO(get_logger(), "Released command interface: %s", interface_name.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Requested to stop unknown interface: %s", interface_name.c_str());
    }
  }

  // Claim start interfaces
  for (const auto & interface_name : start_interfaces) {
    auto it = command_interfaces_.find(interface_name);
    if (it != command_interfaces_.end()) {
      if (it->second.state.claimed) {
        RCLCPP_ERROR(get_logger(), "Interface %s is already claimed!", interface_name.c_str());
        return hardware_interface::return_type::ERROR;
      }
      it->second.state.claimed = true;

      // the CM seems to give an out-range 0.0 command on the first
      // cycle after claiming, so we set a delay to ignore it
      it->second.state.post_claim_delay = 1;

      RCLCPP_INFO(get_logger(), "Claimed command interface: %s", interface_name.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Requested to start unknown interface: %s",
          interface_name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn
ServoHardware::validate_and_mark_requested_state_interfaces(
  const hardware_interface::ComponentInfo & component)
{
  for (const auto & state_interface : component.state_interfaces) {
    auto it = state_interfaces_.find(
        make_interface_full_name(component.name, state_interface.name));
    if (it == state_interfaces_.end()) {
      RCLCPP_ERROR(get_logger(),
                   "State interface '%s' for joint '%s' is not defined in "
                   "state_interfaces_",
                   state_interface.name.c_str(), component.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (it->second.state.requested) {
      RCLCPP_ERROR(get_logger(),
                   "State interface '%s' for joint '%s' was already requested",
                   state_interface.name.c_str(), component.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    it->second.state.requested = true;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ServoHardware::validate_and_mark_requested_command_interfaces(
  const hardware_interface::ComponentInfo & component)
{
  for (const auto & command_interface : component.command_interfaces) {
    auto it = command_interfaces_.find(
        make_interface_full_name(component.name, command_interface.name));
    if (it == command_interfaces_.end()) {
      RCLCPP_ERROR(get_logger(),
                   "Command interface '%s' for joint '%s' is not defined in "
                   "command_interfaces_",
                   command_interface.name.c_str(), component.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (it->second.state.requested) {
      RCLCPP_ERROR(
          get_logger(),
          "Command interface '%s' for joint '%s' was already requested",
          command_interface.name.c_str(), component.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (command_interface.name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(get_logger(),
                   "Joint '%s' has unsupported command interface type '%s'. "
                   "Expected '%s'",
                   component.name.c_str(), command_interface.name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    it->second.state.requested = true;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::string ServoHardware::make_interface_full_name(
  const std::string & joint_name, const std::string & interface_name) const
{
  return prefix_ + joint_name + "/" + interface_name;
}

void ServoHardware::populate_state_definitions()
{
  state_interfaces_[make_interface_full_name(
      "joint_0", hardware_interface::HW_IF_POSITION)] = {
    {},
    [this]() {return servo_0_state_;},
  };
  state_interfaces_[make_interface_full_name(
      "joint_1", hardware_interface::HW_IF_POSITION)] = {
    {},
    [this]() {return servo_1_state_;},
  };
  state_interfaces_[make_interface_full_name(
      "joint_2", hardware_interface::HW_IF_POSITION)] = {
    {},
    [this]() {return servo_2_state_;},
  };
  state_interfaces_[make_interface_full_name(
      "joint_3", hardware_interface::HW_IF_POSITION)] = {
    {},
    [this]() {return servo_3_state_;},
  };
}

void ServoHardware::populate_command_definitions()
{
  command_interfaces_[make_interface_full_name(
      "joint_0", hardware_interface::HW_IF_POSITION)] = {
    {},
    [this](double value) {servo_0_command_ = value;},
  };
  command_interfaces_[make_interface_full_name(
      "joint_1", hardware_interface::HW_IF_POSITION)] = {
    {},
    [this](double value) {servo_1_command_ = value;},
  };
  command_interfaces_[make_interface_full_name(
      "joint_2", hardware_interface::HW_IF_POSITION)] = {
    {},
    [this](double value) {servo_2_command_ = value;},
  };
  command_interfaces_[make_interface_full_name(
      "joint_3", hardware_interface::HW_IF_POSITION)] = {
    {},
    [this](double value) {servo_3_command_ = value;},
  };
}

void ServoHardware::background_thread_function()
{
  RCLCPP_INFO(get_logger(), "Background thread started");
  std::unique_lock<std::mutex> lock(cv_mutex_);
  std::vector<uint8_t> buffer;  // Reusable buffer to avoid repeated allocations

  while (!halting_) {
    cv_.wait_for(lock, std::chrono::milliseconds(10),
      [this]() {return halting_.load();});

    if (halting_) {
      RCLCPP_INFO(get_logger(), "Halting background thread");
      break;
    }

    // Process all queued commands
    if (serial_port_ && protocol_decoder_) {
      servo_protocol_decoder::ServoCommandPayload command_packet;
      while (command_queue_->pop(command_packet)) {
        // Encode packet and send
        buffer = protocol_decoder_->encodeServoCommandPacket(command_packet);
        serial_port_->send(buffer.data(), buffer.size());
      }
    }
  }
}

}  // namespace servo_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(servo_hardware::ServoHardware,
                       hardware_interface::SystemInterface)
