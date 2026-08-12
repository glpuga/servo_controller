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

#include "servo_protocol_decoder/servo_protocol_decoder.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "servo_protocol_decoder/crc16.hpp"
#include "servo_protocol_decoder/packet_buffer_builder.hpp"

namespace servo_protocol_decoder
{

void ServoProtocolDecoder::processStream(
  const std::uint8_t * data, std::size_t size)
{
  // add the new data to the buffer
  in_buffer_.insert(in_buffer_.end(), data, data + size);

  std::size_t offset = 0;
  while (offset < in_buffer_.size()) {
    auto results = validatePacket(
      in_buffer_.data() + offset,
      in_buffer_.data() + in_buffer_.size());

    if (results == PacketValidationResult::VALID) {
      // process the valid packet
      auto bytes_consumed = decodeBufferById(
        in_buffer_.data() + offset,
        in_buffer_.data() + in_buffer_.size());
      // advance the offset by the number of bytes consumed
      offset += bytes_consumed;
    } else if (results == PacketValidationResult::INVALID) {
      // move the buffer forward by one byte and try again
      offset++;
    } else {
      // incomplete packet, break and wait for more data
      break;
    }
  }

  // remove the processed bytes from the buffer
  if (offset > 0) {
    in_buffer_.erase(in_buffer_.begin(), in_buffer_.begin() + offset);
  }
}

std::size_t ServoProtocolDecoder::getPacketSize(
  PacketType packet_type) const
{
  switch (packet_type) {
    case PacketType::SERVO_STATUS:
      return 2 * 4;  // 4 servo positions (2 bytes each)
    case PacketType::SERVO_COMMAND:
      return 2 * 4;  // 4 servo positions (2 bytes each)
  }
  return 0;  // unknown packet type
}

bool ServoProtocolDecoder::isValidPacketType(PacketType packet_type) const
{
  return packet_type == PacketType::SERVO_STATUS ||
         packet_type == PacketType::SERVO_COMMAND;
}

ServoProtocolDecoder::PacketValidationResult
ServoProtocolDecoder::validatePacket(
  const std::uint8_t * start,
  const std::uint8_t * end) const
{
  const auto buffer_size = static_cast<std::size_t>(end - start);

  // check if the buffer has enough data for a packet
  if (buffer_size < 1) {
    return PacketValidationResult::INCOMPLETE;
  }

  // check if the first byte is the sync byte (packet type)
  const auto packet_type = static_cast<std::uint8_t>(start[0]);
  if (packet_type != 0xff) {
    // invalid packet type
    return PacketValidationResult::INVALID;
  }


  // check if the packet type is valid
  if (buffer_size < 2) {
    return PacketValidationResult::INCOMPLETE;
  }

  // check the packet type
  const auto received_packet_type = static_cast<PacketType>(start[1]);

  if (!isValidPacketType(received_packet_type)) {
    return PacketValidationResult::INVALID;
  }

  // can we decode the CRC?
  const auto payload_calculation_size = getPacketSize(received_packet_type);

  if (buffer_size < payload_calculation_size + header_size_ + crc_size_) {
    return PacketValidationResult::INCOMPLETE;
  }

  // check the CRC. The CRC includes the packet type and the payload (skips only sync byte)
  auto received_crc =
    (static_cast<std::uint16_t>(start[header_size_ + payload_calculation_size]) << 8) |
    static_cast<std::uint16_t>(start[header_size_ + payload_calculation_size + 1]);
  auto crc_data_start = start + 1;  // Skip sync byte, include packet type
  auto crc_data_end = crc_data_start + 1 + payload_calculation_size;  // packet type + payload

  auto calculated_crc =
    calculateCRC16(crc_data_start, crc_data_end);

  return received_crc == calculated_crc ? PacketValidationResult::VALID :
         PacketValidationResult::INVALID;
}

PacketBuffer ServoProtocolDecoder::encodeServoStatusPacket(
  const ServoStatusPayload & packet)
{
  // Lambda to normalize radians to 0-2π range, scale by 10000, and round
  auto encode_angle = [](double radians) -> std::uint16_t {
      double normalized = std::fmod(radians, 2.0 * M_PI);
      if (normalized < 0.0) {
        normalized += 2.0 * M_PI;
      }
      return static_cast<std::uint16_t>(std::round(normalized * 10000.0));
    };

  auto builder = PacketBufferBuilder{};

  // Build packet: sync byte + packet type + payload
  builder
  .push(static_cast<std::uint8_t>(0xFF))    // Sync byte
  .push(static_cast<std::uint8_t>(PacketType::SERVO_STATUS))
  .push(encode_angle(packet.servo[0]))
  .push(encode_angle(packet.servo[1]))
  .push(encode_angle(packet.servo[2]))
  .push(encode_angle(packet.servo[3]));

  auto buffer = builder.data();

  // Calculate CRC over packet_type + payload (skip sync byte at index 0)
  std::uint16_t crc = calculateCRC16(buffer.data() + 1, buffer.data() + buffer.size());

  // Append CRC16 (big-endian)
  buffer.push_back(static_cast<std::uint8_t>(crc >> 8));
  buffer.push_back(static_cast<std::uint8_t>(crc & 0xFF));

  return buffer;
}

PacketBuffer ServoProtocolDecoder::encodeServoCommandPacket(
  const ServoCommandPayload & packet)
{
  // Lambda to normalize radians to 0-2π range, scale by 10000, and round
  auto encode_angle = [](double radians) -> std::uint16_t {
      double normalized =
        std::clamp(std::fmod(radians, 2.0 * M_PI), 0.0, 2.0 * M_PI);
      return static_cast<std::uint16_t>(std::round(normalized * 10000.0));
    };

  auto builder = PacketBufferBuilder{};

  // Build packet: sync byte + packet type + payload
  builder
  .push(static_cast<std::uint8_t>(0xFF))    // Sync byte
  .push(static_cast<std::uint8_t>(PacketType::SERVO_COMMAND))
  .push(encode_angle(packet.servo[0]))
  .push(encode_angle(packet.servo[1]))
  .push(encode_angle(packet.servo[2]))
  .push(encode_angle(packet.servo[3]));

  auto buffer = builder.data();

  // Calculate CRC over packet_type + payload (skip sync byte at index 0)
  std::uint16_t crc = calculateCRC16(buffer.data() + 1, buffer.data() + buffer.size());

  // Append CRC16 (big-endian)
  buffer.push_back(static_cast<std::uint8_t>(crc >> 8));
  buffer.push_back(static_cast<std::uint8_t>(crc & 0xFF));

  return buffer;
}

void ServoProtocolDecoder::setServoStatusCallback(ServoStatusCallback callback)
{
  servo_status_callback_ = std::move(callback);
}

void ServoProtocolDecoder::setServoCommandCallback(ServoCommandCallback callback)
{
  servo_command_callback_ = std::move(callback);
}

std::size_t ServoProtocolDecoder::decodeBufferById(
  const std::uint8_t * start,
  const std::uint8_t * end)
{
  // Assumes buffer has been validated and has at least 2 bytes
  const auto packet_type = static_cast<PacketType>(start[1]);
  const auto payload_size = getPacketSize(packet_type);
  const auto total_packet_size = header_size_ + payload_size + crc_size_;

  switch (packet_type) {
    case PacketType::SERVO_STATUS: {
        auto packet = decodeServoStatusPacket(start, end);
        if (packet && servo_status_callback_) {
          servo_status_callback_(*packet);
        }
        break;
      }
    case PacketType::SERVO_COMMAND: {
        auto packet = decodeServoCommandPacket(start, end);
        if (packet && servo_command_callback_) {
          servo_command_callback_(*packet);
        }
        break;
      }
  }

  return total_packet_size;
}

std::optional<ServoStatusPayload> ServoProtocolDecoder::decodeServoStatusPacket(
  const std::uint8_t * start, const std::uint8_t * end) const
{
  const auto expected_size = header_size_ + 2 * 4;  // 4 servo positions, 2 bytes each
  const auto buffer_size = static_cast<std::size_t>(end - start);

  if (buffer_size < expected_size) {
    return std::nullopt;
  }

  ServoStatusPayload packet;

  // Decode 4 servo positions (each is 2 bytes, big-endian, scaled by 10000)
  for (size_t i = 0; i < 4; ++i) {
    const auto offset = header_size_ + i * 2;
    const std::uint16_t raw_value =
      (static_cast<std::uint16_t>(start[offset]) << 8) |
      static_cast<std::uint16_t>(start[offset + 1]);
    packet.servo[i] = static_cast<double>(raw_value) / 10000.0;
  }

  return packet;
}

std::optional<ServoCommandPayload> ServoProtocolDecoder::decodeServoCommandPacket(
  const std::uint8_t * start, const std::uint8_t * end) const
{
  const auto expected_size = header_size_ + 2 * 4;  // 4 servo positions, 2 bytes each
  const auto buffer_size = static_cast<std::size_t>(end - start);

  if (buffer_size < expected_size) {
    return std::nullopt;
  }

  ServoCommandPayload packet;

  // Decode 4 servo positions (each is 2 bytes, big-endian, scaled by 10000)
  for (size_t i = 0; i < 4; ++i) {
    const auto offset = header_size_ + i * 2;
    const std::uint16_t raw_value =
      (static_cast<std::uint16_t>(start[offset]) << 8) |
      static_cast<std::uint16_t>(start[offset + 1]);
    packet.servo[i] = static_cast<double>(raw_value) / 10000.0;
  }

  return packet;
}

}  // namespace servo_protocol_decoder
