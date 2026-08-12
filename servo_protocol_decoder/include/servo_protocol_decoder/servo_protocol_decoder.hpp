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

#ifndef SERVO_PROTOCOL_DECODER__SERVO_PROTOCOL_DECODER_HPP_
#define SERVO_PROTOCOL_DECODER__SERVO_PROTOCOL_DECODER_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "servo_protocol_decoder/datatypes.hpp"
#include "servo_protocol_decoder/packets.hpp"

namespace servo_protocol_decoder
{

class ServoProtocolDecoder
{
public:
  using ServoStatusCallback = std::function<void (const ServoStatusPayload &)>;
  using ServoCommandCallback = std::function<void (const ServoCommandPayload &)>;

  void processStream(const std::uint8_t * data, std::size_t size);

  PacketBuffer encodeServoStatusPacket(const ServoStatusPayload & packet);

  PacketBuffer encodeServoCommandPacket(const ServoCommandPayload & packet);

  void setServoStatusCallback(ServoStatusCallback callback);

  void setServoCommandCallback(ServoCommandCallback callback);

private:
  enum class PacketValidationResult { VALID, INVALID, INCOMPLETE };

  static constexpr std::size_t header_size_ = 2;  // 1 byte sync + 1 byte packet type
  static constexpr std::size_t crc_size_ = 2;     // CRC16 is 2 bytes

  PacketBuffer in_buffer_;

  ServoStatusCallback servo_status_callback_;
  ServoCommandCallback servo_command_callback_;

  PacketValidationResult validatePacket(const std::uint8_t * start, const std::uint8_t * end) const;

  std::size_t getPacketSize(PacketType packet_type) const;

  bool isValidPacketType(PacketType packet_type) const;

  std::size_t decodeBufferById(const std::uint8_t * start, const std::uint8_t * end);

  std::optional<ServoStatusPayload> decodeServoStatusPacket(
    const std::uint8_t * start,
    const std::uint8_t * end) const;

  std::optional<ServoCommandPayload> decodeServoCommandPacket(
    const std::uint8_t * start,
    const std::uint8_t * end) const;
};

}  // namespace servo_protocol_decoder

#endif  // SERVO_PROTOCOL_DECODER__SERVO_PROTOCOL_DECODER_HPP_
