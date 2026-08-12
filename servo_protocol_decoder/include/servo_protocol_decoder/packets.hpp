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

#ifndef SERVO_PROTOCOL_DECODER__PACKETS_HPP_
#define SERVO_PROTOCOL_DECODER__PACKETS_HPP_

#include <array>

namespace servo_protocol_decoder
{

enum class PacketType : std::uint8_t
{
  SERVO_COMMAND = 0x00,
  SERVO_STATUS = 0x28,
};

struct ServoStatusPayload
{
  std::array<double, 4> servo;  // radians
};

struct ServoCommandPayload
{
  std::array<double, 4> servo;    // radians
};

}  // namespace servo_protocol_decoder

#endif  // SERVO_PROTOCOL_DECODER__PACKETS_HPP_
