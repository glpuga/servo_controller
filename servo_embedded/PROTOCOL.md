# Serial Communication Protocol

This document describes the binary serial protocol used for communication with the ESP32-C3 servo controller.

## Overview

- **Baud Rate**: 115200
- **Protocol**: Binary packet-based with CRC32 validation
- **Packet Size**: 14 bytes (fixed)
- **Byte Order**: Big-endian for data, Little-endian for CRC

## Packet Structure

All packets follow this structure:

```
┌──────────┬──────────┬───────────────────┬──────────────┐
│ Start    │ Cmd ID   │ Payload (8 bytes) │ CRC32        │
│ 0xFF     │ 1 byte   │                   │ 4 bytes (LE) │
├──────────┼──────────┼───────────────────┼──────────────┤
│ Byte 0   │ Byte 1   │ Bytes 2-9         │ Bytes 10-13  │
└──────────┴──────────┴───────────────────┴──────────────┘
```

### Fields

1. **Start Byte** (1 byte): Always `0xFF` - Used for packet synchronization
2. **Command ID** (1 byte): Identifies the packet type (see Command IDs below)
3. **Payload** (8 bytes): Command-specific data
4. **CRC32** (4 bytes, little-endian): CRC32 checksum over bytes 1-9 (Command ID + Payload)

### CRC Calculation

- **Algorithm**: CRC32-LE (IEEE 802.3 polynomial)
- **Covers**: Bytes 1-9 (Command ID + 8-byte payload)
- **Implementation**: ESP32 ROM `crc32_le(0, data, 9)`
- **Byte Order**: Little-endian (LSB first)

## Command IDs

| ID | Name          | Direction     | Description                    |
|----|---------------|---------------|--------------------------------|
| 0  | SERVO_COMMAND | Host → ESP32  | Set target servo positions     |
| 40 | SERVO_STATUS  | ESP32 → Host  | Report current servo positions |

## Command Details

### 0x00 - SERVO_COMMAND (Incoming)

Sets the target positions for all four servos. The profiler task will smooth the motion to respect velocity limits.

**Payload Format:**

```
┌──────┬────────────┬────────────┬────────────┬────────────┐
│ ID   │ Servo 0    │ Servo 1    │ Servo 2    │ Servo 3    │
│ 0x00 │ 2 bytes BE │ 2 bytes BE │ 2 bytes BE │ 2 bytes BE │
└──────┴────────────┴────────────┴────────────┴────────────┘
```

**Servo Value Encoding:**
- **Type**: 16-bit unsigned integer (big-endian)
- **Format**: Fixed-point with 2 decimal places
- **Calculation**: `angle_degrees × 100`
- **Range**: 0-18000 (representing 0.00° to 180.00°)
- **Examples**:
  - 0° → `0x0000` (0)
  - 90.00° → `0x2328` (9000)
  - 90.25° → `0x233D` (9025)
  - 180.00° → `0x4650` (18000)

**Example Packet (All servos to 90°):**

```
Byte   Value  Description
────────────────────────────────
  0    0xFF   Start byte
  1    0x00   Command ID (SERVO_COMMAND)
  2    0x23   Servo 0 high byte (9000 >> 8)
  3    0x28   Servo 0 low byte (9000 & 0xFF)
  4    0x23   Servo 1 high byte
  5    0x28   Servo 1 low byte
  6    0x23   Servo 2 high byte
  7    0x28   Servo 2 low byte
  8    0x23   Servo 3 high byte
  9    0x28   Servo 3 low byte
 10    0xXX   CRC byte 0 (LSB)
 11    0xXX   CRC byte 1
 12    0xXX   CRC byte 2
 13    0xXX   CRC byte 3 (MSB)
```

### 0x28 - SERVO_STATUS (Outgoing)

Reports the current positions of all four servos. This is the actual position after velocity profiling, not necessarily the target.

**Payload Format:**

```
┌──────┬────────────┬────────────┬────────────┬────────────┐
│ ID   │ Servo 0    │ Servo 1    │ Servo 2    │ Servo 3    │
│ 0x28 │ 2 bytes BE │ 2 bytes BE │ 2 bytes BE │ 2 bytes BE │
└──────┴────────────┴────────────┴────────────┴────────────┘
```

**Encoding**: Same as SERVO_COMMAND (fixed-point with 2 decimal places)

## Motion Profiling

The ESP32 implements velocity-limited motion profiling:

- **Update Rate**: 20ms (50 Hz)
- **Max Velocity**: 300°/s (configurable via `MG996R_CONFIG.maxAngularRate`)
- **Max Step**: 6° per update (300°/s × 0.02s)

When a SERVO_COMMAND is received:
1. Target positions are placed in the profiler queue
2. The profiler task smooths motion to respect velocity limits
3. Current positions are sent to the servo task every 20ms
4. Servos move smoothly without jerky motion

## Error Handling

### Invalid Packets

Packets are **silently discarded** if:
- CRC32 validation fails
- Unknown Command ID is received
- Malformed data (handled by state machine reset)

### State Machine

The decoder uses a 3-state machine:
1. **WAITING_FOR_START**: Looking for `0xFF`
2. **READING_PAYLOAD**: Collecting 9 bytes (ID + data)
3. **READING_CRC**: Collecting 4 CRC bytes

After any complete packet (valid or invalid), the state resets to `WAITING_FOR_START`.

## Implementation Notes

### Data Types

- **Servo Angles**: Stored internally as `double` (degrees)
- **Wire Format**: 16-bit unsigned integers (fixed-point × 100)
- **Pulse Width**: 16-bit unsigned integers (microseconds, 500-2500µs)

### Hardware Mapping

| Servo | GPIO Pin | Description            |
|-------|----------|------------------------|
| 0     | GPIO0    | Servo position 0       |
| 1     | GPIO1    | Servo position 1       |
| 2     | GPIO3    | Servo position 2       |
| 3     | GPIO10   | Servo position 3       |

### Servo Specifications (MG996R)

- **Pulse Width Range**: 500µs - 2500µs
- **Angular Range**: 0° - 180°
- **Max Speed**: 300°/s (60° in 0.2s at 4.8V)

## Example Code

### Python: Send SERVO_COMMAND

```python
import serial
import struct
from zlib import crc32

def send_servo_command(ser, angles):
    """Send servo command with 4 angles (degrees)."""
    # Start byte
    packet = bytearray([0xFF])

    # Command ID
    packet.append(0x00)

    # Pack angles as big-endian 16-bit values (fixed-point × 100)
    for angle in angles:
        value = int(angle * 100)
        packet.extend(struct.pack('>H', value))  # Big-endian uint16

    # Calculate CRC32 (little-endian)
    crc = crc32(packet[1:10]) & 0xFFFFFFFF
    packet.extend(struct.pack('<I', crc))  # Little-endian uint32

    # Send
    ser.write(packet)

# Example usage
with serial.Serial('/dev/ttyUSB0', 115200) as ser:
    send_servo_command(ser, [90.0, 45.0, 135.0, 60.0])
```

### Python: Receive SERVO_STATUS

```python
def decode_servo_status(packet):
    """Decode a SERVO_STATUS packet."""
    if len(packet) != 14 or packet[0] != 0xFF:
        return None

    # Verify CRC
    payload = packet[1:10]
    received_crc = struct.unpack('<I', packet[10:14])[0]
    calculated_crc = crc32(payload) & 0xFFFFFFFF

    if received_crc != calculated_crc:
        return None  # CRC mismatch

    if packet[1] != 0x28:  # SERVO_STATUS ID
        return None

    # Decode servo angles
    angles = []
    for i in range(4):
        offset = 2 + i * 2
        value = struct.unpack('>H', packet[offset:offset+2])[0]
        angles.append(value / 100.0)

    return angles

# Example usage
status = decode_servo_status(received_packet)
if status:
    print(f"Servos: {status[0]:.2f}°, {status[1]:.2f}°, {status[2]:.2f}°, {status[3]:.2f}°")
```

## Timing Characteristics

- **Serial Baud Rate**: 115200 bps
- **Packet Size**: 14 bytes = 112 bits
- **Transmission Time**: ~1ms per packet
- **Profiler Update**: Every 20ms
- **Command Latency**: < 20ms (next profiler update)
- **Motion Time**: Depends on distance and 300°/s limit

## Version History

- **v1.0** (2026-08-08): Initial protocol definition
  - SERVO_COMMAND (0x00) and SERVO_STATUS (0x28)
  - CRC32 validation
  - Fixed 14-byte packets
  - Big-endian data, little-endian CRC
