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

#include <cmath>
#include <vector>

#include "servo_protocol_decoder/servo_protocol_decoder.hpp"
#include "servo_protocol_decoder/packets.hpp"
#include "servo_protocol_decoder/datatypes.hpp"
#include "servo_protocol_decoder/crc16.hpp"

namespace servo_protocol_decoder
{

// Tolerance for comparing decoded values - matches encoding precision of 1/10000 radians
constexpr double kDecodingTolerance = 0.0001;

// Helper function to encode a radian value the same way the encoder does
std::uint16_t encodeRadianValue(double radians)
{
  double normalized = std::fmod(radians, 2.0 * M_PI);
  if (normalized < 0.0) {
    normalized += 2.0 * M_PI;
  }
  return static_cast<std::uint16_t>(std::round(normalized * 10000.0));
}

// Helper function to get the high byte of an encoded value
std::uint8_t getHighByte(std::uint16_t value)
{
  return static_cast<std::uint8_t>(value >> 8);
}

// Helper function to get the low byte of an encoded value
std::uint8_t getLowByte(std::uint16_t value)
{
  return static_cast<std::uint8_t>(value & 0xFF);
}

class ServoProtocolDecoderTest : public ::testing::Test {};

TEST_F(ServoProtocolDecoderTest, EncodeServoCommandPacketBasic) {
  ServoProtocolDecoder decoder;

  ServoCommandPayload packet;
  packet.servo[0] = 0.0;          // 0 radians
  packet.servo[1] = M_PI / 2.0;   // π/2 radians
  packet.servo[2] = M_PI;         // π radians
  packet.servo[3] = 3.0 * M_PI / 2.0;  // 3π/2 radians

  const auto buffer = decoder.encodeServoCommandPacket(packet);

  // Expected packet structure:
  // [0]: 0xFF (sync byte)
  // [1]: 0x00 (SERVO_COMMAND packet type)
  // [2-3]: servo[0] = 0.0 * 10000 = 0 = 0x0000
  // [4-5]: servo[1] = (π/2) * 10000 ≈ 15708 = 0x3D5C
  // [6-7]: servo[2] = π * 10000 ≈ 31416 = 0x7AB8
  // [8-9]: servo[3] = (3π/2) * 10000 ≈ 47124 = 0xB814
  // [10-11]: CRC16 (calculated over bytes [1-9])

  ASSERT_EQ(buffer.size(), 12u);

  // Check sync byte
  EXPECT_EQ(buffer[0], 0xFF);

  // Check packet type
  EXPECT_EQ(buffer[1], 0x00);

  // Check servo values using helper functions
  const auto servo0_encoded = encodeRadianValue(packet.servo[0]);
  EXPECT_EQ(buffer[2], getHighByte(servo0_encoded));
  EXPECT_EQ(buffer[3], getLowByte(servo0_encoded));

  const auto servo1_encoded = encodeRadianValue(packet.servo[1]);
  EXPECT_EQ(buffer[4], getHighByte(servo1_encoded));
  EXPECT_EQ(buffer[5], getLowByte(servo1_encoded));

  const auto servo2_encoded = encodeRadianValue(packet.servo[2]);
  EXPECT_EQ(buffer[6], getHighByte(servo2_encoded));
  EXPECT_EQ(buffer[7], getLowByte(servo2_encoded));

  const auto servo3_encoded = encodeRadianValue(packet.servo[3]);
  EXPECT_EQ(buffer[8], getHighByte(servo3_encoded));
  EXPECT_EQ(buffer[9], getLowByte(servo3_encoded));

  // Verify CRC is calculated over packet_type + payload (bytes 1-9)
  const std::uint16_t expected_crc = calculateCRC16(buffer.data() + 1, buffer.data() + 10);
  const std::uint16_t actual_crc = (static_cast<std::uint16_t>(buffer[10]) << 8) |
    static_cast<std::uint16_t>(buffer[11]);

  EXPECT_EQ(actual_crc, expected_crc);
}

TEST_F(ServoProtocolDecoderTest, EncodeServoCommandPacketZeroValues) {
  ServoProtocolDecoder decoder;

  ServoCommandPayload packet;
  packet.servo[0] = 0.0;
  packet.servo[1] = 0.0;
  packet.servo[2] = 0.0;
  packet.servo[3] = 0.0;

  const auto buffer = decoder.encodeServoCommandPacket(packet);

  ASSERT_EQ(buffer.size(), 12u);

  // Check sync byte and packet type
  EXPECT_EQ(buffer[0], 0xFF);
  EXPECT_EQ(buffer[1], 0x00);

  // Check all servo values using helper functions
  for (int i = 0; i < 4; i++) {
    const auto encoded = encodeRadianValue(packet.servo[i]);
    EXPECT_EQ(buffer[2 + i * 2], getHighByte(encoded));
    EXPECT_EQ(buffer[3 + i * 2], getLowByte(encoded));
  }

  // Verify CRC
  const std::uint16_t expected_crc = calculateCRC16(buffer.data() + 1, buffer.data() + 10);
  const std::uint16_t actual_crc = (static_cast<std::uint16_t>(buffer[10]) << 8) |
    static_cast<std::uint16_t>(buffer[11]);

  EXPECT_EQ(actual_crc, expected_crc);
}

TEST_F(ServoProtocolDecoderTest, EncodeServoCommandPacketMaxValues) {
  ServoProtocolDecoder decoder;

  ServoCommandPayload packet;
  // Maximum value in 0-2π range is just under 2π
  // 2π * 10000 ≈ 62832, which fits in uint16_t (max 65535)
  packet.servo[0] = 2.0 * M_PI - 0.0001;  // Just under 2π
  packet.servo[1] = 2.0 * M_PI - 0.0001;
  packet.servo[2] = 2.0 * M_PI - 0.0001;
  packet.servo[3] = 2.0 * M_PI - 0.0001;

  const auto buffer = decoder.encodeServoCommandPacket(packet);

  ASSERT_EQ(buffer.size(), 12u);

  // Check sync byte and packet type
  EXPECT_EQ(buffer[0], 0xFF);
  EXPECT_EQ(buffer[1], 0x00);

  // All servos should encode to the same near-maximum value
  for (int i = 0; i < 4; i++) {
    const auto encoded = encodeRadianValue(packet.servo[i]);
    EXPECT_EQ(buffer[2 + i * 2], getHighByte(encoded));
    EXPECT_EQ(buffer[3 + i * 2], getLowByte(encoded));
  }

  // Verify CRC
  const std::uint16_t expected_crc = calculateCRC16(buffer.data() + 1, buffer.data() + 10);
  const std::uint16_t actual_crc = (static_cast<std::uint16_t>(buffer[10]) << 8) |
    static_cast<std::uint16_t>(buffer[11]);

  EXPECT_EQ(actual_crc, expected_crc);
}

TEST_F(ServoProtocolDecoderTest, EncodeServoCommandPacketMixedValues) {
  ServoProtocolDecoder decoder;

  ServoCommandPayload packet;
  packet.servo[0] = 0.0;        // 0 radians
  packet.servo[1] = M_PI / 4.0;  // π/4 radians
  packet.servo[2] = M_PI;       // π radians
  packet.servo[3] = 5.0 * M_PI / 4.0;  // 5π/4 radians

  const auto buffer = decoder.encodeServoCommandPacket(packet);

  ASSERT_EQ(buffer.size(), 12u);

  // Check sync byte and packet type
  EXPECT_EQ(buffer[0], 0xFF);
  EXPECT_EQ(buffer[1], 0x00);

  // Check each servo value using helper functions
  for (int i = 0; i < 4; i++) {
    const auto encoded = encodeRadianValue(packet.servo[i]);
    EXPECT_EQ(buffer[2 + i * 2], getHighByte(encoded));
    EXPECT_EQ(buffer[3 + i * 2], getLowByte(encoded));
  }

  // Verify CRC
  const std::uint16_t expected_crc = calculateCRC16(buffer.data() + 1, buffer.data() + 10);
  const std::uint16_t actual_crc = (static_cast<std::uint16_t>(buffer[10]) << 8) |
    static_cast<std::uint16_t>(buffer[11]);

  EXPECT_EQ(actual_crc, expected_crc);
}

TEST_F(ServoProtocolDecoderTest, EncodeServoCommandPacketDecimalPrecision) {
  ServoProtocolDecoder decoder;

  ServoCommandPayload packet;
  packet.servo[0] = 0.1234;  // Small radian value
  packet.servo[1] = 1.5708;  // ≈ π/2
  packet.servo[2] = 3.1416;  // ≈ π
  packet.servo[3] = 4.7124;  // ≈ 3π/2

  const auto buffer = decoder.encodeServoCommandPacket(packet);

  ASSERT_EQ(buffer.size(), 12u);

  // Check sync byte and packet type
  EXPECT_EQ(buffer[0], 0xFF);
  EXPECT_EQ(buffer[1], 0x00);

  // Check each servo value using helper functions
  for (int i = 0; i < 4; i++) {
    const auto encoded = encodeRadianValue(packet.servo[i]);
    EXPECT_EQ(buffer[2 + i * 2], getHighByte(encoded));
    EXPECT_EQ(buffer[3 + i * 2], getLowByte(encoded));
  }

  // Verify CRC
  const std::uint16_t expected_crc = calculateCRC16(buffer.data() + 1, buffer.data() + 10);
  const std::uint16_t actual_crc = (static_cast<std::uint16_t>(buffer[10]) << 8) |
    static_cast<std::uint16_t>(buffer[11]);

  EXPECT_EQ(actual_crc, expected_crc);
}

TEST_F(ServoProtocolDecoderTest, DecodeServoCommandPacketEmptyBuffer) {
  ServoProtocolDecoder decoder;

  bool callback_invoked = false;
  ServoCommandPayload decoded_packet;

  decoder.setServoCommandCallback(
    [&](const ServoCommandPayload & packet) {
      callback_invoked = true;
      decoded_packet = packet;
    });

  // Process empty buffer
  std::vector<std::uint8_t> empty_buffer;
  decoder.processStream(empty_buffer.data(), empty_buffer.size());

  // Callback should not be invoked
  EXPECT_FALSE(callback_invoked);
}

TEST_F(ServoProtocolDecoderTest, DecodeServoCommandPacketWholeBuffer) {
  ServoProtocolDecoder decoder;

  // Create original packet with radian values
  ServoCommandPayload original_packet;
  original_packet.servo[0] = 0.5;     // radians
  original_packet.servo[1] = 1.5708;  // ≈ π/2
  original_packet.servo[2] = 3.1416;  // ≈ π
  original_packet.servo[3] = 4.7124;  // ≈ 3π/2

  // Encode the packet
  const auto encoded_buffer = decoder.encodeServoCommandPacket(original_packet);

  // Set up callback to capture decoded packet
  bool callback_invoked = false;
  ServoCommandPayload decoded_packet;

  decoder.setServoCommandCallback(
    [&](const ServoCommandPayload & packet) {
      callback_invoked = true;
      decoded_packet = packet;
    });

  // Process the whole buffer at once
  std::vector<std::uint8_t> buffer(encoded_buffer.begin(), encoded_buffer.end());
  decoder.processStream(buffer.data(), buffer.size());

  // Verify callback was invoked
  ASSERT_TRUE(callback_invoked);

  // Verify decoded packet matches original
  EXPECT_NEAR(decoded_packet.servo[0], original_packet.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[1], original_packet.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[2], original_packet.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[3], original_packet.servo[3], kDecodingTolerance);
}

TEST_F(ServoProtocolDecoderTest, DecodeServoCommandPacketOneByteAtATime) {
  ServoProtocolDecoder decoder;

  // Create original packet with radian values
  ServoCommandPayload original_packet;
  original_packet.servo[0] = 0.1234;
  original_packet.servo[1] = 1.5708;
  original_packet.servo[2] = 3.1416;
  original_packet.servo[3] = 4.7124;

  // Encode the packet
  const auto encoded_buffer = decoder.encodeServoCommandPacket(original_packet);

  // Set up callback to capture decoded packet
  bool callback_invoked = false;
  ServoCommandPayload decoded_packet;

  decoder.setServoCommandCallback(
    [&](const ServoCommandPayload & packet) {
      callback_invoked = true;
      decoded_packet = packet;
    });

  // Process the buffer one byte at a time
  for (const auto byte : encoded_buffer) {
    std::vector<std::uint8_t> single_byte = {byte};
    decoder.processStream(single_byte.data(), single_byte.size());
  }

  // Verify callback was invoked
  ASSERT_TRUE(callback_invoked);

  // Verify decoded packet matches original
  EXPECT_NEAR(decoded_packet.servo[0], original_packet.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[1], original_packet.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[2], original_packet.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[3], original_packet.servo[3], kDecodingTolerance);
}

TEST_F(ServoProtocolDecoderTest, DecodeServoCommandPacketWithGarbagePrefix) {
  ServoProtocolDecoder decoder;

  // Create original packet with radian values
  ServoCommandPayload original_packet;
  original_packet.servo[0] = 0.5;
  original_packet.servo[1] = 1.5708;
  original_packet.servo[2] = 3.1416;
  original_packet.servo[3] = 4.7124;

  // Encode the packet
  const auto encoded_buffer = decoder.encodeServoCommandPacket(original_packet);

  // Set up callback to capture decoded packet
  bool callback_invoked = false;
  ServoCommandPayload decoded_packet;

  decoder.setServoCommandCallback(
    [&](const ServoCommandPayload & packet) {
      callback_invoked = true;
      decoded_packet = packet;
    });

  // Create buffer with garbage prefix
  std::vector<std::uint8_t> buffer = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
  buffer.insert(buffer.end(), encoded_buffer.begin(), encoded_buffer.end());

  decoder.processStream(buffer.data(), buffer.size());

  // Verify callback was invoked
  ASSERT_TRUE(callback_invoked);

  // Verify decoded packet matches original
  EXPECT_NEAR(decoded_packet.servo[0], original_packet.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[1], original_packet.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[2], original_packet.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[3], original_packet.servo[3], kDecodingTolerance);
}

TEST_F(ServoProtocolDecoderTest, DecodeServoCommandPacketWithGarbageSuffix) {
  ServoProtocolDecoder decoder;

  // Create original packet with radian values
  ServoCommandPayload original_packet;
  original_packet.servo[0] = 0.0;       // 0 radians
  original_packet.servo[1] = M_PI / 2;  // π/2 radians
  original_packet.servo[2] = M_PI;      // π radians
  original_packet.servo[3] = 3 * M_PI / 2;  // 3π/2 radians

  // Encode the packet
  const auto encoded_buffer = decoder.encodeServoCommandPacket(original_packet);

  // Set up callback to capture decoded packet
  bool callback_invoked = false;
  ServoCommandPayload decoded_packet;

  decoder.setServoCommandCallback(
    [&](const ServoCommandPayload & packet) {
      callback_invoked = true;
      decoded_packet = packet;
    });

  // Create buffer with garbage suffix
  std::vector<std::uint8_t> buffer(encoded_buffer.begin(), encoded_buffer.end());
  std::vector<std::uint8_t> garbage = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
  buffer.insert(buffer.end(), garbage.begin(), garbage.end());

  decoder.processStream(buffer.data(), buffer.size());

  // Verify callback was invoked
  ASSERT_TRUE(callback_invoked);

  // Verify decoded packet matches original
  EXPECT_NEAR(decoded_packet.servo[0], original_packet.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[1], original_packet.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[2], original_packet.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[3], original_packet.servo[3], kDecodingTolerance);
}

TEST_F(ServoProtocolDecoderTest, DecodeMultipleServoCommandPackets) {
  ServoProtocolDecoder decoder;

  // Create two different packets with radian values
  ServoCommandPayload packet1;
  packet1.servo[0] = 0.0;
  packet1.servo[1] = M_PI / 4;
  packet1.servo[2] = M_PI / 2;
  packet1.servo[3] = 3 * M_PI / 4;

  ServoCommandPayload packet2;
  packet2.servo[0] = M_PI;
  packet2.servo[1] = 5 * M_PI / 4;
  packet2.servo[2] = 3 * M_PI / 2;
  packet2.servo[3] = 7 * M_PI / 4;

  // Encode both packets
  const auto encoded_buffer1 = decoder.encodeServoCommandPacket(packet1);
  const auto encoded_buffer2 = decoder.encodeServoCommandPacket(packet2);

  // Set up callback to capture decoded packets
  std::vector<ServoCommandPayload> decoded_packets;

  decoder.setServoCommandCallback(
    [&](const ServoCommandPayload & packet) {
      decoded_packets.push_back(packet);
    });

  // Create buffer with both packets concatenated
  std::vector<std::uint8_t> buffer(encoded_buffer1.begin(), encoded_buffer1.end());
  buffer.insert(buffer.end(), encoded_buffer2.begin(), encoded_buffer2.end());

  decoder.processStream(buffer.data(), buffer.size());

  // Verify both callbacks were invoked
  ASSERT_EQ(decoded_packets.size(), 2u);

  // Verify first packet
  EXPECT_NEAR(decoded_packets[0].servo[0], packet1.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[0].servo[1], packet1.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[0].servo[2], packet1.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[0].servo[3], packet1.servo[3], kDecodingTolerance);

  // Verify second packet
  EXPECT_NEAR(decoded_packets[1].servo[0], packet2.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[1].servo[1], packet2.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[1].servo[2], packet2.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[1].servo[3], packet2.servo[3], kDecodingTolerance);
}

TEST_F(ServoProtocolDecoderTest, DecodeMultiplePacketsWithGarbage) {
  ServoProtocolDecoder decoder;

  // Create three different packets with radian values
  ServoCommandPayload packet1;
  packet1.servo[0] = 0.1;
  packet1.servo[1] = 0.5;
  packet1.servo[2] = 1.0;
  packet1.servo[3] = 1.5;

  ServoCommandPayload packet2;
  packet2.servo[0] = 2.0;
  packet2.servo[1] = 2.5;
  packet2.servo[2] = 3.0;
  packet2.servo[3] = 3.5;

  ServoCommandPayload packet3;
  packet3.servo[0] = 4.0;
  packet3.servo[1] = 4.5;
  packet3.servo[2] = 5.0;
  packet3.servo[3] = 5.5;

  // Encode all packets
  const auto encoded_buffer1 = decoder.encodeServoCommandPacket(packet1);
  const auto encoded_buffer2 = decoder.encodeServoCommandPacket(packet2);
  const auto encoded_buffer3 = decoder.encodeServoCommandPacket(packet3);

  // Set up callback to capture decoded packets
  std::vector<ServoCommandPayload> decoded_packets;

  decoder.setServoCommandCallback(
    [&](const ServoCommandPayload & packet) {
      decoded_packets.push_back(packet);
    });

  // Create buffer with garbage before, between, and after packets
  std::vector<std::uint8_t> buffer = {0xDE, 0xAD, 0xBE, 0xEF};  // Garbage prefix
  buffer.insert(buffer.end(), encoded_buffer1.begin(), encoded_buffer1.end());

  std::vector<std::uint8_t> garbage_mid1 = {0xCA, 0xFE};  // Garbage between packet 1 and 2
  buffer.insert(buffer.end(), garbage_mid1.begin(), garbage_mid1.end());
  buffer.insert(buffer.end(), encoded_buffer2.begin(), encoded_buffer2.end());

  std::vector<std::uint8_t> garbage_mid2 = {0xBA, 0xBE, 0xF0, 0x0D};  // Garbage
  buffer.insert(buffer.end(), garbage_mid2.begin(), garbage_mid2.end());
  buffer.insert(buffer.end(), encoded_buffer3.begin(), encoded_buffer3.end());

  std::vector<std::uint8_t> garbage_suffix = {0x00, 0x11, 0x22};  // Garbage suffix
  buffer.insert(buffer.end(), garbage_suffix.begin(), garbage_suffix.end());

  decoder.processStream(buffer.data(), buffer.size());

  // Verify all three callbacks were invoked
  ASSERT_EQ(decoded_packets.size(), 3u);

  // Verify first packet
  EXPECT_NEAR(decoded_packets[0].servo[0], packet1.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[0].servo[1], packet1.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[0].servo[2], packet1.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[0].servo[3], packet1.servo[3], kDecodingTolerance);

  // Verify second packet
  EXPECT_NEAR(decoded_packets[1].servo[0], packet2.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[1].servo[1], packet2.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[1].servo[2], packet2.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[1].servo[3], packet2.servo[3], kDecodingTolerance);

  // Verify third packet
  EXPECT_NEAR(decoded_packets[2].servo[0], packet3.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[2].servo[1], packet3.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[2].servo[2], packet3.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_packets[2].servo[3], packet3.servo[3], kDecodingTolerance);
}

// ServoStatus callback tests

TEST_F(ServoProtocolDecoderTest, EncodeServoStatusPacketBasic) {
  ServoProtocolDecoder decoder;

  ServoStatusPayload packet;
  packet.servo[0] = M_PI / 6;     // π/6 radians (30°)
  packet.servo[1] = M_PI / 2;     // π/2 radians (90°)
  packet.servo[2] = 3 * M_PI / 4;  // 3π/4 radians (135°)
  packet.servo[3] = M_PI;         // π radians (180°)

  const auto buffer = decoder.encodeServoStatusPacket(packet);

  ASSERT_EQ(buffer.size(), 12u);

  // Check sync byte
  EXPECT_EQ(buffer[0], 0xFF);

  // Check packet type (SERVO_STATUS = 0x28)
  EXPECT_EQ(buffer[1], 0x28);

  // Check all servo values using helper functions
  for (int i = 0; i < 4; i++) {
    const auto encoded = encodeRadianValue(packet.servo[i]);
    EXPECT_EQ(buffer[2 + i * 2], getHighByte(encoded));
    EXPECT_EQ(buffer[3 + i * 2], getLowByte(encoded));
  }

  // Verify CRC is calculated over packet_type + payload (bytes 1-9)
  const std::uint16_t expected_crc = calculateCRC16(buffer.data() + 1, buffer.data() + 10);
  const std::uint16_t actual_crc = (static_cast<std::uint16_t>(buffer[10]) << 8) |
    static_cast<std::uint16_t>(buffer[11]);

  EXPECT_EQ(actual_crc, expected_crc);
}

TEST_F(ServoProtocolDecoderTest, DecodeServoStatusPacketWholeBuffer) {
  ServoProtocolDecoder decoder;

  // Create original packet with radian values
  ServoStatusPayload original_packet;
  original_packet.servo[0] = 0.5;
  original_packet.servo[1] = 1.5708;
  original_packet.servo[2] = 3.1416;
  original_packet.servo[3] = 4.7124;

  // Encode the packet
  const auto encoded_buffer = decoder.encodeServoStatusPacket(original_packet);

  // Set up callback to capture decoded packet
  bool callback_invoked = false;
  ServoStatusPayload decoded_packet;

  decoder.setServoStatusCallback(
    [&](const ServoStatusPayload & packet) {
      callback_invoked = true;
      decoded_packet = packet;
    });

  // Process the whole buffer at once
  std::vector<std::uint8_t> buffer(encoded_buffer.begin(), encoded_buffer.end());
  decoder.processStream(buffer.data(), buffer.size());

  // Verify callback was invoked
  ASSERT_TRUE(callback_invoked);

  // Verify decoded packet matches original
  EXPECT_NEAR(decoded_packet.servo[0], original_packet.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[1], original_packet.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[2], original_packet.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[3], original_packet.servo[3], kDecodingTolerance);
}

TEST_F(ServoProtocolDecoderTest, DecodeServoStatusPacketOneByteAtATime) {
  ServoProtocolDecoder decoder;

  // Create original packet with radian values
  ServoStatusPayload original_packet;
  original_packet.servo[0] = 0.234;
  original_packet.servo[1] = 1.567;
  original_packet.servo[2] = 2.890;
  original_packet.servo[3] = 4.234;

  // Encode the packet
  const auto encoded_buffer = decoder.encodeServoStatusPacket(original_packet);

  // Set up callback to capture decoded packet
  bool callback_invoked = false;
  ServoStatusPayload decoded_packet;

  decoder.setServoStatusCallback(
    [&](const ServoStatusPayload & packet) {
      callback_invoked = true;
      decoded_packet = packet;
    });

  // Process the buffer one byte at a time
  for (const auto byte : encoded_buffer) {
    std::vector<std::uint8_t> single_byte = {byte};
    decoder.processStream(single_byte.data(), single_byte.size());
  }

  // Verify callback was invoked
  ASSERT_TRUE(callback_invoked);

  // Verify decoded packet matches original
  EXPECT_NEAR(decoded_packet.servo[0], original_packet.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[1], original_packet.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[2], original_packet.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[3], original_packet.servo[3], kDecodingTolerance);
}

TEST_F(ServoProtocolDecoderTest, DecodeServoStatusPacketWithGarbage) {
  ServoProtocolDecoder decoder;

  // Create original packet with radian values
  ServoStatusPayload original_packet;
  original_packet.servo[0] = 1.0;
  original_packet.servo[1] = 2.0;
  original_packet.servo[2] = 3.0;
  original_packet.servo[3] = 4.5;

  // Encode the packet
  const auto encoded_buffer = decoder.encodeServoStatusPacket(original_packet);

  // Set up callback to capture decoded packet
  bool callback_invoked = false;
  ServoStatusPayload decoded_packet;

  decoder.setServoStatusCallback(
    [&](const ServoStatusPayload & packet) {
      callback_invoked = true;
      decoded_packet = packet;
    });

  // Create buffer with garbage prefix and suffix
  std::vector<std::uint8_t> buffer = {0xAA, 0xBB, 0xCC};
  buffer.insert(buffer.end(), encoded_buffer.begin(), encoded_buffer.end());
  std::vector<std::uint8_t> suffix = {0xDD, 0xEE, 0xFF};
  buffer.insert(buffer.end(), suffix.begin(), suffix.end());

  decoder.processStream(buffer.data(), buffer.size());

  // Verify callback was invoked
  ASSERT_TRUE(callback_invoked);

  // Verify decoded packet matches original
  EXPECT_NEAR(decoded_packet.servo[0], original_packet.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[1], original_packet.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[2], original_packet.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_packet.servo[3], original_packet.servo[3], kDecodingTolerance);
}

TEST_F(ServoProtocolDecoderTest, DecodeMixedServoStatusAndCommandPackets) {
  ServoProtocolDecoder decoder;

  // Create status and command packets with radian values
  ServoStatusPayload status_packet;
  status_packet.servo[0] = 0.111;
  status_packet.servo[1] = 1.222;
  status_packet.servo[2] = 2.333;
  status_packet.servo[3] = 3.444;

  ServoCommandPayload command_packet;
  command_packet.servo[0] = 1.555;
  command_packet.servo[1] = 2.666;
  command_packet.servo[2] = 3.777;
  command_packet.servo[3] = 4.888;

  // Encode both packets
  const auto encoded_status = decoder.encodeServoStatusPacket(status_packet);
  const auto encoded_command = decoder.encodeServoCommandPacket(command_packet);

  // Set up callbacks to capture decoded packets
  bool status_callback_invoked = false;
  bool command_callback_invoked = false;
  ServoStatusPayload decoded_status;
  ServoCommandPayload decoded_command;

  decoder.setServoStatusCallback(
    [&](const ServoStatusPayload & packet) {
      status_callback_invoked = true;
      decoded_status = packet;
    });

  decoder.setServoCommandCallback(
    [&](const ServoCommandPayload & packet) {
      command_callback_invoked = true;
      decoded_command = packet;
    });

  // Create buffer with both packet types and garbage
  std::vector<std::uint8_t> buffer = {0xDE, 0xAD};
  buffer.insert(buffer.end(), encoded_status.begin(), encoded_status.end());
  std::vector<std::uint8_t> garbage = {0xBE, 0xEF};
  buffer.insert(buffer.end(), garbage.begin(), garbage.end());
  buffer.insert(buffer.end(), encoded_command.begin(), encoded_command.end());

  decoder.processStream(buffer.data(), buffer.size());

  // Verify both callbacks were invoked
  ASSERT_TRUE(status_callback_invoked);
  ASSERT_TRUE(command_callback_invoked);

  // Verify decoded status packet
  EXPECT_NEAR(decoded_status.servo[0], status_packet.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_status.servo[1], status_packet.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_status.servo[2], status_packet.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_status.servo[3], status_packet.servo[3], kDecodingTolerance);

  // Verify decoded command packet
  EXPECT_NEAR(decoded_command.servo[0], command_packet.servo[0], kDecodingTolerance);
  EXPECT_NEAR(decoded_command.servo[1], command_packet.servo[1], kDecodingTolerance);
  EXPECT_NEAR(decoded_command.servo[2], command_packet.servo[2], kDecodingTolerance);
  EXPECT_NEAR(decoded_command.servo[3], command_packet.servo[3], kDecodingTolerance);
}

}  // namespace servo_protocol_decoder
