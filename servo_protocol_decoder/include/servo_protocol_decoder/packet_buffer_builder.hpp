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

#ifndef SERVO_PROTOCOL_DECODER__PACKET_BUFFER_BUILDER_HPP_
#define SERVO_PROTOCOL_DECODER__PACKET_BUFFER_BUILDER_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "servo_protocol_decoder/packets.hpp"
#include "servo_protocol_decoder/datatypes.hpp"

namespace servo_protocol_decoder
{

class PacketBufferBuilder
{
public:
  PacketBufferBuilder() = default;
  explicit PacketBufferBuilder(PacketBuffer && buffer)
  : buffer_(std::move(buffer)) {}

  PacketBufferBuilder & push(std::uint8_t value) &
  {
    buffer_.push_back(value);
    return *this;
  }

  PacketBufferBuilder && push(std::uint8_t value) &&
  {
    buffer_.push_back(value);
    return std::move(*this);
  }

  PacketBufferBuilder & push(std::int8_t value) &
  {
    return push(static_cast<std::uint8_t>(value));
  }

  PacketBufferBuilder && push(std::int8_t value) &&
  {
    return std::move(*this).push(static_cast<std::uint8_t>(value));
  }

  PacketBufferBuilder & push(std::uint16_t value) &
  {
    buffer_.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));  // High byte (big-endian)
    buffer_.push_back(static_cast<std::uint8_t>(value & 0xFF));         // Low byte
    return *this;
  }

  PacketBufferBuilder && push(std::uint16_t value) &&
  {
    buffer_.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));  // High byte (big-endian)
    buffer_.push_back(static_cast<std::uint8_t>(value & 0xFF));         // Low byte
    return std::move(*this);
  }

  PacketBufferBuilder & push(std::int16_t value) &
  {
    return push(static_cast<std::uint16_t>(value));
  }

  PacketBufferBuilder && push(std::int16_t value) &&
  {
    return std::move(*this).push(static_cast<std::uint16_t>(value));
  }

  PacketBufferBuilder & push(std::uint32_t value) &
  {
    buffer_.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));  // Highest byte
    buffer_.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
    buffer_.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    buffer_.push_back(static_cast<std::uint8_t>(value & 0xFF));          // Lowest byte
    return *this;
  }

  PacketBufferBuilder && push(std::uint32_t value) &&
  {
    buffer_.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFF));  // Highest byte
    buffer_.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFF));
    buffer_.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFF));
    buffer_.push_back(static_cast<std::uint8_t>(value & 0xFF));          // Lowest byte
    return std::move(*this);
  }

  PacketBufferBuilder & push(std::int32_t value) &
  {
    return push(static_cast<std::uint32_t>(value));
  }

  PacketBufferBuilder && push(std::int32_t value) &&
  {
    return std::move(*this).push(static_cast<std::uint32_t>(value));
  }

  PacketBuffer & data() & {return buffer_;}

  PacketBuffer build() const & {return buffer_;}

  PacketBuffer build() && {return std::move(buffer_);}

private:
  PacketBuffer buffer_;
};

}  // namespace servo_protocol_decoder

#endif  // SERVO_PROTOCOL_DECODER__PACKET_BUFFER_BUILDER_HPP_
