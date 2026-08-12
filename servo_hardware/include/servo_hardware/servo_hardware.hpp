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

#ifndef SERVO_HARDWARE__SERVO_HARDWARE_HPP_
#define SERVO_HARDWARE__SERVO_HARDWARE_HPP_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
#include <unordered_set>
#include <unordered_map>

#include "realtime_tools/lock_free_queue.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "servo_hardware/serial_port.hpp"
#include "servo_protocol_decoder/servo_protocol_decoder.hpp"
#include "servo_protocol_decoder/packets.hpp"

namespace servo_hardware
{
class ServoHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ServoHardware)

  /**
   * \brief Initialization of the hardware interface from data parsed from the
   * robot's URDF. \param params Hardware component interface parameters
   * containing executor and HardwareInfo. \return CallbackReturn::SUCCESS if
   * initialization was successful, CallbackReturn::ERROR otherwise.
   *
   * This method is called once during the initialization phase. It should:
   * - Initialize all member variables
   * - Process parameters from the HardwareInfo structure
   * - Validate that all required parameters are present and valid
   */
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
  override;

  /**
   * \brief Configuration of the hardware interface.
   * \param previous_state The previous lifecycle state.
   * \return CallbackReturn::SUCCESS if configuration was successful,
   * CallbackReturn::ERROR otherwise.
   *
   * This method is called when the hardware component is configured. It should:
   * - Setup communication to the hardware
   * - Prepare everything so that the hardware can be activated
   * - Allocate resources needed for communication
   */
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * \brief Activation of the hardware interface.
   * \param previous_state The previous lifecycle state.
   * \return CallbackReturn::SUCCESS if activation was successful,
   * CallbackReturn::ERROR otherwise.
   *
   * This method is called when the hardware component is activated. It should:
   * - Enable hardware "power"
   * - Start any background processes needed for operation
   * - Prepare the hardware to accept commands
   */
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * \brief Deactivation of the hardware interface.
   * \param previous_state The previous lifecycle state.
   * \return CallbackReturn::SUCCESS if deactivation was successful,
   * CallbackReturn::ERROR otherwise.
   *
   * This method is called when the hardware component is deactivated. It
   * should:
   * - Disable hardware "power"
   * - Stop any background processes
   * - Put hardware in a safe state
   */
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * \brief Read the current state from the hardware.
   * \param time Current time.
   * \param period Time elapsed since the last read.
   * \return return_type::OK if the read was successful, return_type::ERROR
   * otherwise.
   *
   * This method is called periodically to get the states from the hardware and
   * store them to internal variables that were defined in
   * export_state_interfaces. This method must be real-time safe.
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * \brief Write commands to the hardware.
   * \param time Current time.
   * \param period Time elapsed since the last write.
   * \return return_type::OK if the write was successful, return_type::ERROR
   * otherwise.
   *
   * This method is called periodically to command the hardware based on the
   * values stored in internal variables that were defined in
   * export_command_interfaces. This method must be real-time safe.
   */
  hardware_interface::return_type
  write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * \brief Handle command mode switching.
   * \param start_interfaces List of command interfaces to claim.
   * \param stop_interfaces List of command interfaces to release.
   * \return return_type::OK if the mode switch was successful, return_type::ERROR
   * otherwise.
   *
   * This method is called when controllers are loaded/unloaded or when the
   * command mode changes. It updates the claimed status of command interfaces.
   */
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  // Interface definition initialization
  void populate_state_definitions();
  void populate_command_definitions();

  hardware_interface::CallbackReturn
  validate_and_mark_requested_state_interfaces(
    const hardware_interface::ComponentInfo & component);

  hardware_interface::CallbackReturn
  validate_and_mark_requested_command_interfaces(
    const hardware_interface::ComponentInfo & component);

  std::string make_interface_full_name(
    const std::string & joint_name,
    const std::string & interface_name) const;

  struct StateInterfaceState
  {
    bool requested{false};  // Whether this interface was requested in URDF
  };

  // Interface data structures
  struct StateInterfaceData
  {
    StateInterfaceState state;  // State of the state interface
    std::function<double()> get_value;  // Functor to retrieve current state value
  };

  struct CommandInterfaceState
  {
    bool requested{false};  // Whether this interface was requested in URDF
    bool claimed{false};  // Whether this interface is currently claimed by a controller

    // This is necessary because there seems to be a bug when the CM
    // enforces command limits, in that if 0.0 is not in the range of
    // the command limits, before the command interface is claimed it will
    // stil get valued commands. After being claimed, the first iteration
    // will still get a 0.0, and only from that point on will the command
    // values be clipped to the correct range.
    std::uint32_t post_claim_delay{0};
  };

  struct CommandInterfaceData
  {
    CommandInterfaceState state;  // State of the command interface
    std::function<void(double)> set_command;  // Functor to send command to hardware
  };

  // Hardware parameters
  std::string device_;
  std::string prefix_;
  uint32_t baudrate_;

  // Interface maps
  std::unordered_map<std::string, StateInterfaceData> state_interfaces_;
  std::unordered_map<std::string, CommandInterfaceData> command_interfaces_;

  // State storage
  double servo_0_state_;
  double servo_1_state_;
  double servo_2_state_;
  double servo_3_state_;

  double servo_0_command_;
  double servo_1_command_;
  double servo_2_command_;
  double servo_3_command_;

  // Serial communication
  std::unique_ptr<servo_hardware::SerialPort> serial_port_;
  std::unique_ptr<servo_protocol_decoder::ServoProtocolDecoder> protocol_decoder_;
  std::unique_ptr<realtime_tools::RealtimeThreadSafeBox<servo_protocol_decoder::ServoStatusPayload>>
  status_box_;

  // Background thread for async operations
  void background_thread_function();
  std::thread background_thread_;
  std::atomic<bool> halting_{false};
  std::atomic<bool> state_message_received_{false};
  std::condition_variable cv_;
  std::mutex cv_mutex_;
  std::unique_ptr<realtime_tools::LockFreeSPSCQueue<servo_protocol_decoder::ServoCommandPayload>>
  command_queue_;
};

}  // namespace servo_hardware

#endif  // SERVO_HARDWARE__SERVO_HARDWARE_HPP_
