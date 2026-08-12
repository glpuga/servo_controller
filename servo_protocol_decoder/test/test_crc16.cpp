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

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "servo_protocol_decoder/crc16.hpp"

namespace servo_protocol_decoder
{

class CRC16Test : public ::testing::Test {};

TEST_F(CRC16Test, EmptyData) {
  // CRC16-CCITT-FALSE of empty data
  std::vector<std::uint8_t> data;
  const auto crc = calculateCRC16(data.data(), data.data() + data.size());

  EXPECT_EQ(crc, 0xFFFF);
}

TEST_F(CRC16Test, SingleByte) {
  // Test CRC16-CCITT-FALSE with a single byte
  std::vector<std::uint8_t> data = {0x00};
  const auto crc = calculateCRC16(data.data(), data.data() + data.size());

  EXPECT_EQ(crc, 0xE1F0);
}

TEST_F(CRC16Test, SingleByteFF) {
  // Test CRC16-CCITT-FALSE with a single 0xFF byte
  std::vector<std::uint8_t> data = {0xFF};
  const auto crc = calculateCRC16(data.data(), data.data() + data.size());

  EXPECT_EQ(crc, 0xFF00);
}

TEST_F(CRC16Test, KnownASCIIString) {
  // Test with a known ASCII string "123456789"
  // This is a standard test vector for CRC16-CCITT-FALSE
  std::vector<std::uint8_t> data = {'1', '2', '3', '4', '5',
    '6', '7', '8', '9'};
  const std::uint32_t crc = calculateCRC16(data.data(), data.data() + data.size());

  EXPECT_EQ(crc, 0x29B1);
}

TEST_F(CRC16Test, AnotherKnownString) {
  // Test with "The quick brown fox jumps over the lazy dog"
  const std::string str = "The quick brown fox jumps over the lazy dog";
  const std::vector<std::uint8_t> data(str.begin(), str.end());
  const auto crc = calculateCRC16(data.data(), data.data() + data.size());

  EXPECT_EQ(crc, 0x8FDD);
}

TEST_F(CRC16Test, AllZeros) {
  // Test with multiple zero bytes
  const std::vector<std::uint8_t> data(10, 0x00);
  const auto crc = calculateCRC16(data.data(), data.data() + data.size());

  EXPECT_EQ(crc, 0xE139);
}

TEST_F(CRC16Test, AllOnes) {
  // Test with multiple 0xFF bytes
  const std::vector<std::uint8_t> data(10, 0xFF);
  const auto crc = calculateCRC16(data.data(), data.data() + data.size());

  EXPECT_EQ(crc, 0xA6E1);
}

TEST_F(CRC16Test, SequentialBytes) {
  // Test with sequential bytes 0-255
  std::vector<std::uint8_t> data(256);
  for (auto i = 0; i < 256; ++i) {
    data[i] = static_cast<std::uint8_t>(i);
  }
  const auto crc = calculateCRC16(data.data(), data.data() + data.size());

  EXPECT_EQ(crc, 0x3FBD);
}

TEST_F(CRC16Test, DifferentDataDifferentCRC) {
  // Ensure different data produces different CRC
  const std::vector<std::uint8_t> data1 = {0x01, 0x02, 0x03};
  const std::vector<std::uint8_t> data2 = {0x01, 0x02, 0x04};

  const auto crc1 = calculateCRC16(data1.data(), data1.data() + data1.size());
  const auto crc2 = calculateCRC16(data2.data(), data2.data() + data2.size());

  EXPECT_NE(crc1, crc2);
}

TEST_F(CRC16Test, OrderMatters) {
  // Ensure byte order matters
  const std::vector<std::uint8_t> data1 = {0x01, 0x02, 0x03};
  const std::vector<std::uint8_t> data2 = {0x03, 0x02, 0x01};

  const auto crc1 = calculateCRC16(data1.data(), data1.data() + data1.size());
  const auto crc2 = calculateCRC16(data2.data(), data2.data() + data2.size());

  EXPECT_NE(crc1, crc2);
}

TEST_F(CRC16Test, SameDataSameCRC) {
  // Ensure same data produces same CRC (deterministic)
  const std::vector<std::uint8_t> data = {0xAB, 0xCD, 0xEF, 0x12, 0x34};

  const auto crc1 = calculateCRC16(data.data(), data.data() + data.size());
  const auto crc2 = calculateCRC16(data.data(), data.data() + data.size());

  EXPECT_EQ(crc1, crc2);
}

TEST_F(CRC16Test, LargeData) {
  // Test with larger data set (1KB)
  std::vector<std::uint8_t> data(1024);
  for (auto i = 0u; i < data.size(); ++i) {
    data[i] = static_cast<std::uint8_t>(i & 0xFF);
  }

  const auto crc = calculateCRC16(data.data(), data.data() + data.size());
  // 1KB of repeating pattern 0-255
  EXPECT_EQ(crc, 0x758F);
}

TEST_F(CRC16Test, TwoBytePayload) {
  // Test with a small 2-byte payload
  const std::vector<std::uint8_t> data = {0xAB, 0xCD};
  const auto crc = calculateCRC16(data.data(), data.data() + data.size());

  EXPECT_EQ(crc, 0xD46A);
}

TEST_F(CRC16Test, SingleBitDifference) {
  // Verify that a single bit difference changes the CRC
  const std::vector<std::uint8_t> data1 = {0x00, 0x00, 0x00};
  const std::vector<std::uint8_t> data2 = {0x01, 0x00,
    0x00};                                        // One bit different

  const auto crc1 = calculateCRC16(data1.data(), data1.data() + data1.size());
  const auto crc2 = calculateCRC16(data2.data(), data2.data() + data2.size());

  EXPECT_NE(crc1, crc2);
}

}    // namespace servo_protocol_decoder
