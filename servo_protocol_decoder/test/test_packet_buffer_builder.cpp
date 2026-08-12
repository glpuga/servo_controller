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

#include <vector>

#include "servo_protocol_decoder/datatypes.hpp"
#include "servo_protocol_decoder/packet_buffer_builder.hpp"

namespace servo_protocol_decoder
{

class PacketBufferBuilderTest : public ::testing::Test {};

TEST_F(PacketBufferBuilderTest, DefaultConstruction) {
  PacketBufferBuilder builder;
  const auto buffer = builder.build();

  EXPECT_TRUE(buffer.empty());
}

TEST_F(PacketBufferBuilderTest, ConstructionWithExistingBuffer) {
  PacketBuffer initial_buffer = {0x01, 0x02, 0x03};
  PacketBufferBuilder builder(std::move(initial_buffer));
  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 3u);
  EXPECT_EQ(buffer[0], 0x01);
  EXPECT_EQ(buffer[1], 0x02);
  EXPECT_EQ(buffer[2], 0x03);
}

TEST_F(PacketBufferBuilderTest, PushUint8) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::uint8_t>(0x42));
  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 1u);
  EXPECT_EQ(buffer[0], 0x42);
}

TEST_F(PacketBufferBuilderTest, PushInt8) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::int8_t>(-1));
  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 1u);
  EXPECT_EQ(buffer[0], 0xFF);
}

TEST_F(PacketBufferBuilderTest, PushInt8Positive) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::int8_t>(127));
  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 1u);
  EXPECT_EQ(buffer[0], 0x7F);
}

TEST_F(PacketBufferBuilderTest, PushUint16) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::uint16_t>(0x1234));
  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 2u);
  // Big-endian byte order
  EXPECT_EQ(buffer[0], 0x12);
  EXPECT_EQ(buffer[1], 0x34);
}

TEST_F(PacketBufferBuilderTest, PushInt16) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::int16_t>(-1));
  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 2u);
  EXPECT_EQ(buffer[0], 0xFF);
  EXPECT_EQ(buffer[1], 0xFF);
}

TEST_F(PacketBufferBuilderTest, PushInt16Positive) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::int16_t>(0x7ABC));
  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 2u);
  // Big-endian byte order
  EXPECT_EQ(buffer[0], 0x7A);
  EXPECT_EQ(buffer[1], 0xBC);
}

TEST_F(PacketBufferBuilderTest, PushUint32) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::uint32_t>(0x12345678));
  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 4u);
  // Big-endian byte order
  EXPECT_EQ(buffer[0], 0x12);
  EXPECT_EQ(buffer[1], 0x34);
  EXPECT_EQ(buffer[2], 0x56);
  EXPECT_EQ(buffer[3], 0x78);
}

TEST_F(PacketBufferBuilderTest, PushInt32) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::int32_t>(-1));
  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 4u);
  EXPECT_EQ(buffer[0], 0xFF);
  EXPECT_EQ(buffer[1], 0xFF);
  EXPECT_EQ(buffer[2], 0xFF);
  EXPECT_EQ(buffer[3], 0xFF);
}

TEST_F(PacketBufferBuilderTest, PushInt32Positive) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::int32_t>(0x7ABCDEF0));
  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 4u);
  // Big-endian byte order
  EXPECT_EQ(buffer[0], 0x7A);
  EXPECT_EQ(buffer[1], 0xBC);
  EXPECT_EQ(buffer[2], 0xDE);
  EXPECT_EQ(buffer[3], 0xF0);
}

TEST_F(PacketBufferBuilderTest, ChainedPushOperations) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::uint8_t>(0x01))
  .push(static_cast<std::uint16_t>(0x0203))
  .push(static_cast<std::uint32_t>(0x04050607));

  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 7u);
  EXPECT_EQ(buffer[0], 0x01);
  EXPECT_EQ(buffer[1], 0x02);
  EXPECT_EQ(buffer[2], 0x03);
  EXPECT_EQ(buffer[3], 0x04);
  EXPECT_EQ(buffer[4], 0x05);
  EXPECT_EQ(buffer[5], 0x06);
  EXPECT_EQ(buffer[6], 0x07);
}

TEST_F(PacketBufferBuilderTest, MixedTypesPush) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::uint8_t>(0xAA))
  .push(static_cast<std::int8_t>(-2))
  .push(static_cast<std::uint16_t>(0xBBCC))
  .push(static_cast<std::int16_t>(-3))
  .push(static_cast<std::uint32_t>(0xDDEEFF00))
  .push(static_cast<std::int32_t>(-4));

  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 14u);
  // uint8
  EXPECT_EQ(buffer[0], 0xAA);
  // int8 (-2)
  EXPECT_EQ(buffer[1], 0xFE);
  // uint16 (0xBBCC) - big-endian
  EXPECT_EQ(buffer[2], 0xBB);
  EXPECT_EQ(buffer[3], 0xCC);
  // int16 (-3 = 0xFFFD) - big-endian
  EXPECT_EQ(buffer[4], 0xFF);
  EXPECT_EQ(buffer[5], 0xFD);
  // uint32 (0xDDEEFF00) - big-endian
  EXPECT_EQ(buffer[6], 0xDD);
  EXPECT_EQ(buffer[7], 0xEE);
  EXPECT_EQ(buffer[8], 0xFF);
  EXPECT_EQ(buffer[9], 0x00);
  // int32 (-4 = 0xFFFFFFFC) - big-endian
  EXPECT_EQ(buffer[10], 0xFF);
  EXPECT_EQ(buffer[11], 0xFF);
  EXPECT_EQ(buffer[12], 0xFF);
  EXPECT_EQ(buffer[13], 0xFC);
}

TEST_F(PacketBufferBuilderTest, MultipleGetBufferCalls) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::uint8_t>(0x11));

  const auto buffer1 = builder.build();
  const auto buffer2 = builder.build();

  EXPECT_EQ(buffer1.size(), buffer2.size());
  EXPECT_EQ(buffer1, buffer2);
}

TEST_F(PacketBufferBuilderTest, ZeroValues) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::uint8_t>(0x00))
  .push(static_cast<std::uint16_t>(0x0000))
  .push(static_cast<std::uint32_t>(0x00000000));

  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 7u);
  for (const auto byte : buffer) {
    EXPECT_EQ(byte, 0x00);
  }
}

TEST_F(PacketBufferBuilderTest, MaxValues) {
  PacketBufferBuilder builder;
  builder.push(static_cast<std::uint8_t>(0xFF))
  .push(static_cast<std::uint16_t>(0xFFFF))
  .push(static_cast<std::uint32_t>(0xFFFFFFFF));

  const auto buffer = builder.build();

  ASSERT_EQ(buffer.size(), 7u);
  for (const auto byte : buffer) {
    EXPECT_EQ(byte, 0xFF);
  }
}

TEST_F(PacketBufferBuilderTest, RvalueConstruction) {
  // Test using builder as an rvalue - buffer should be moved
  auto buffer = PacketBufferBuilder()
    .push(static_cast<std::uint8_t>(0xAA))
    .push(static_cast<std::uint16_t>(0xBBCC))
    .push(static_cast<std::uint32_t>(0xDDEEFF00))
    .build();

  ASSERT_EQ(buffer.size(), 7u);
  EXPECT_EQ(buffer[0], 0xAA);
  // uint16 (0xBBCC) - big-endian
  EXPECT_EQ(buffer[1], 0xBB);
  EXPECT_EQ(buffer[2], 0xCC);
  // uint32 (0xDDEEFF00) - big-endian
  EXPECT_EQ(buffer[3], 0xDD);
  EXPECT_EQ(buffer[4], 0xEE);
  EXPECT_EQ(buffer[5], 0xFF);
  EXPECT_EQ(buffer[6], 0x00);
}

TEST_F(PacketBufferBuilderTest, RvalueWithInitialBuffer) {
  // Test using builder as an rvalue with initial buffer
  PacketBuffer initial = {0x01, 0x02};
  auto buffer = PacketBufferBuilder(std::move(initial))
    .push(static_cast<std::uint8_t>(0x03))
    .push(static_cast<std::uint16_t>(0x0405))
    .build();

  ASSERT_EQ(buffer.size(), 5u);
  EXPECT_EQ(buffer[0], 0x01);
  EXPECT_EQ(buffer[1], 0x02);
  EXPECT_EQ(buffer[2], 0x03);
  EXPECT_EQ(buffer[3], 0x04);
  EXPECT_EQ(buffer[4], 0x05);
}

}  // namespace servo_protocol_decoder
