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

#ifndef SERVO_PROTOCOL_DECODER__CRC16_HPP_
#define SERVO_PROTOCOL_DECODER__CRC16_HPP_

#include <cstdint>
#include <vector>

namespace servo_protocol_decoder
{

/// Calculate CRC16-CCITT-FALSE checksum
/// Polynomial: 0x1021, Initial: 0xFFFF, RefIn: false, RefOut: false, XorOut: 0x0000
/// @param start Pointer to the start of data
/// @param end Pointer to the end of data (one past the last byte)
/// @return The CRC16 checksum (16-bit value)
std::uint16_t calculateCRC16(const std::uint8_t * start, const std::uint8_t * end);

}  // namespace servo_protocol_decoder

#endif  // SERVO_PROTOCOL_DECODER__CRC16_HPP_
