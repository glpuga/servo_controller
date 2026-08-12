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

#include <Arduino.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Status LED pin (inverted: LOW=ON, HIGH=OFF)
#define STATUS_LED 8

// Servo Pins (using PWM-capable pins only)
#define SERVO_PIN_0 0   // GPIO0
#define SERVO_PIN_1 1   // GPIO1
#define SERVO_PIN_2 3   // GPIO3
#define SERVO_PIN_3 10  // GPIO10

// LEDC Configuration
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_14_BIT  // 14-bit resolution (0-16383)
#define LEDC_FREQUENCY 50                 // 50 Hz for servos
#define LEDC_CHANNEL_0 LEDC_CHANNEL_0
#define LEDC_CHANNEL_1 LEDC_CHANNEL_1
#define LEDC_CHANNEL_2 LEDC_CHANNEL_2
#define LEDC_CHANNEL_3 LEDC_CHANNEL_3

// Servo initial position (radians, π/2 = 90 degrees)
#define SERVO_INITIAL_POSITION 1.5708

// Profiler update period (milliseconds)
#define PROFILER_UPDATE_PERIOD_MS 20

// Blink period (milliseconds)
#define BLINK_PERIOD_MS 200

// Servo configuration structure
typedef struct
{
  double max_angular_position;  // Maximum angular position in radians
  double min_angular_position;  // Minimum angular position in radians
  int minPulseWidth_us;         // Minimum pulse width in microseconds
  int maxPulseWidth_us;         // Maximum pulse width in microseconds
  double maxAngularRate;        // Maximum angular rate in radians/second
} ServoConfig;

// MG996R servo configuration
const ServoConfig MG996R_CONFIG = {
  .max_angular_position = M_PI,    // 180 degrees
  .min_angular_position = 0.0,     // 0 degrees
  .minPulseWidth_us = 500,         // 0.5ms for 0 degrees
  .maxPulseWidth_us = 2500,        // 2.5ms for 180 degrees
  .maxAngularRate =
    5.236      // 60 degrees / 0.2 seconds = π/3 rad/s ≈ 5.236 rad/s
};

// Servo position structure (angles in radians)
typedef struct
{
  double servo0_angle;
  double servo1_angle;
  double servo2_angle;
  double servo3_angle;
} ServoPositions;

// Forward declarations
void configureLedcChannel(ledc_channel_t channel, int gpio_num);
void setLedcDuty(ledc_channel_t channel, uint32_t duty);
uint32_t anglesToDutyCycle(double angleInRadians);

// Task handles
TaskHandle_t blinkTaskHandle = NULL;
TaskHandle_t serialReaderTaskHandle = NULL;
TaskHandle_t servoTaskHandle = NULL;
TaskHandle_t profilerTaskHandle = NULL;

// Queue handles
QueueHandle_t servoQueue = NULL;     // Profiler -> Servo task
QueueHandle_t profilerQueue = NULL;  // Stream decoder -> Profiler task

// LED state tracker
bool ledState = false;

// Command/Packet IDs
enum class CommandId : uint8_t
{
  SERVO_COMMAND = 0,  // Incoming: Set servo positions
  SERVO_STATUS = 40   // Outgoing: Servo status feedback
};

// Protocol state machine
enum class DecoderState { WAITING_FOR_START, READING_PAYLOAD, READING_CRC };

static DecoderState decoderState = DecoderState::WAITING_FOR_START;
static uint8_t payloadBuffer[9];
static uint8_t crcBuffer[2];
static uint8_t byteIndex = 0;

/**
 * Calculate CRC16-CCITT-FALSE checksum
 * Polynomial: 0x1021, Initial: 0xFFFF, RefIn: false, RefOut: false, XorOut:
 * 0x0000
 * @param data Pointer to data buffer
 * @param length Number of bytes to process
 * @return The CRC16 checksum (16-bit value)
 */
uint16_t calculateCRC16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFF;  // Initial value

  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;  // XOR with high byte

    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;  // Polynomial
      } else {
        crc = crc << 1;
      }
    }
  }

  return crc;  // XorOut = 0x0000 (no final XOR)
}

/**
 * Convert angle in radians to LEDC duty cycle
 * Maps the servo's angular range to PWM duty cycle in timer units
 * @param angleInRadians Angle in radians (0 to π, constrained automatically)
 * @return Duty cycle in timer units (0-16383 for 14-bit resolution)
 */
uint32_t anglesToDutyCycle(double angleInRadians)
{
  const auto constrainedAngle =
    std::min(MG996R_CONFIG.max_angular_position,
               std::max(angleInRadians, MG996R_CONFIG.min_angular_position));

  // Calculate pulse width in microseconds
  const auto pulseRange =
    MG996R_CONFIG.maxPulseWidth_us - MG996R_CONFIG.minPulseWidth_us;
  const auto angleRange =
    MG996R_CONFIG.max_angular_position - MG996R_CONFIG.min_angular_position;

  const auto pulsesPerRadian = static_cast<double>(pulseRange) / angleRange;

  const auto microseconds =
    MG996R_CONFIG.minPulseWidth_us +
    ((constrainedAngle - MG996R_CONFIG.min_angular_position) *
    pulsesPerRadian);

  // Convert microseconds to duty cycle units
  // At 50Hz: period = 20,000 μs
  // duty_cycle = (pulse_width_us / 20000) * (2^14)
  const double period_us = 1000000.0 / LEDC_FREQUENCY;  // 20,000 μs at 50Hz
  const uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;  // 16383 for 14-bit
  const uint32_t duty = static_cast<uint32_t>(
    (microseconds / period_us) * (1 << LEDC_DUTY_RES));

  // Ensure duty cycle is within valid range
  return std::min(duty, max_duty);
}

/**
 * Publish servo state packet over serial
 * Packet format: [0xFF] [ID] [servo0_h] [servo0_l] ... [servo3_h] [servo3_l]
 * [crc0-1]
 * Note: Angles are transmitted as radians * 10000 (e.g., π/2 rad -> 15708)
 * @param positions Current servo positions to publish (in radians internally)
 */
void publishServoState(const ServoPositions & positions)
{
  uint8_t packet[12];
  // Start byte
  packet[0] = 0xFF;

  // Packet ID
  packet[1] = static_cast<uint8_t>(CommandId::SERVO_STATUS);

  // Convert angles from radians to radians * 10000 (e.g., π/2 rad -> 15708)
  uint16_t servo0_fp = (uint16_t)(positions.servo0_angle * 10000);
  uint16_t servo1_fp = (uint16_t)(positions.servo1_angle * 10000);
  uint16_t servo2_fp = (uint16_t)(positions.servo2_angle * 10000);
  uint16_t servo3_fp = (uint16_t)(positions.servo3_angle * 10000);

  // Pack servo values as big-endian 16-bit values
  packet[2] = (servo0_fp >> 8) & 0xFF;  // high byte
  packet[3] = servo0_fp & 0xFF;         // low byte
  packet[4] = (servo1_fp >> 8) & 0xFF;
  packet[5] = servo1_fp & 0xFF;
  packet[6] = (servo2_fp >> 8) & 0xFF;
  packet[7] = servo2_fp & 0xFF;
  packet[8] = (servo3_fp >> 8) & 0xFF;
  packet[9] = servo3_fp & 0xFF;

  // Calculate CRC16 over payload (packet ID + 8 bytes of servo data)
  uint16_t crc = calculateCRC16(&packet[1], 9);

  // Pack CRC16 as big-endian (to match servo data byte order)
  packet[10] = (crc >> 8) & 0xFF;
  packet[11] = (crc) & 0xFF;

  // Send packet over serial
  Serial.write(packet, sizeof(packet));
}

/**
 * Servo control task - receives position commands and updates servos
 */
void vServoTask(void *pvParameters)
{
  ServoPositions positions;

  while (1) {
    // Wait for a position command from the queue
    if (xQueueReceive(servoQueue, &positions, portMAX_DELAY) == pdTRUE) {
      // Convert angles to duty cycles and update servos using LEDC
      setLedcDuty(LEDC_CHANNEL_0, anglesToDutyCycle(positions.servo0_angle));
      setLedcDuty(LEDC_CHANNEL_1, anglesToDutyCycle(positions.servo1_angle));
      setLedcDuty(LEDC_CHANNEL_2, anglesToDutyCycle(positions.servo2_angle));
      setLedcDuty(LEDC_CHANNEL_3, anglesToDutyCycle(positions.servo3_angle));
    }
  }
}

/**
 * Profiler task - smooths servo motion by limiting velocity
 */
void vProfilerTask(void *pvParameters)
{
  ServoPositions currentPositions = {
    SERVO_INITIAL_POSITION, SERVO_INITIAL_POSITION, SERVO_INITIAL_POSITION,
    SERVO_INITIAL_POSITION};

  ServoPositions targetPositions = currentPositions;

  // Calculate max angle change per update period
  // maxAngularRate is in radians/second, update period is in milliseconds
  double maxDeltaPerUpdate =
    MG996R_CONFIG.maxAngularRate * (PROFILER_UPDATE_PERIOD_MS / 1000.0);

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // Check for new target position (non-blocking)
    xQueueReceive(profilerQueue, &targetPositions, 0);

    // Move current positions towards targets, respecting max velocity
    auto moveTowards = [](double current, double target,
      double maxDelta) -> double {
        double delta = target - current;
        return current + std::max(-maxDelta, std::min(delta, maxDelta));
      };

    currentPositions.servo0_angle =
      moveTowards(currentPositions.servo0_angle, targetPositions.servo0_angle,
                    maxDeltaPerUpdate);
    currentPositions.servo1_angle =
      moveTowards(currentPositions.servo1_angle, targetPositions.servo1_angle,
                    maxDeltaPerUpdate);
    currentPositions.servo2_angle =
      moveTowards(currentPositions.servo2_angle, targetPositions.servo2_angle,
                    maxDeltaPerUpdate);
    currentPositions.servo3_angle =
      moveTowards(currentPositions.servo3_angle, targetPositions.servo3_angle,
                    maxDeltaPerUpdate);

    // Send updated positions to servo task
    xQueueSend(servoQueue, &currentPositions, 0);

    // Publish current positions over serial
    publishServoState(currentPositions);

    // Wait for next update period (20ms)
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PROFILER_UPDATE_PERIOD_MS));
  }
}

/**
 * Handle SERVO_COMMAND (ID=0) - parse servo angles and send to profiler
 * Payload format: [ID] [servo0_h] [servo0_l] [servo1_h] [servo1_l] ...
 * Note: Angles are received as radians * 10000 (e.g., π/2 rad -> 15708)
 * @param payload Pointer to 9-byte payload buffer
 */
void handleServoCommand(const uint8_t *payload)
{
  // Extract servo angles from big-endian 16-bit fixed-point values (radians *
  // 10000)
  uint16_t servo0_fp = (payload[1] << 8) | payload[2];
  uint16_t servo1_fp = (payload[3] << 8) | payload[4];
  uint16_t servo2_fp = (payload[5] << 8) | payload[6];
  uint16_t servo3_fp = (payload[7] << 8) | payload[8];

  const auto scaleAndClip = [](uint16_t fixedPointValue) -> double {
    // Convert from fixed-point to radians
      double angle = static_cast<double>(fixedPointValue) / 10000.0;

      return std::max(MG996R_CONFIG.min_angular_position,
                    std::min(angle, MG996R_CONFIG.max_angular_position));
    };

  // Convert from radians * 10000 to radians (e.g., 15708 -> π/2 rad)
  ServoPositions targetPos;
  targetPos.servo0_angle = scaleAndClip(servo0_fp);
  targetPos.servo1_angle = scaleAndClip(servo1_fp);
  targetPos.servo2_angle = scaleAndClip(servo2_fp);
  targetPos.servo3_angle = scaleAndClip(servo3_fp);

  // Send to profiler queue
  xQueueSend(profilerQueue, &targetPos, 0);
}

/**
 * Serial stream decoder - processes incoming serial data
 * Protocol: [0xFF] [9-byte payload] [2-byte CRC16]
 * Payload: [ID] [8 bytes data]
 * @param byte The received byte to process
 */
void serialStreamDecoder(uint8_t byte)
{
  switch (decoderState) {
    case DecoderState::WAITING_FOR_START:
      if (byte == 0xFF) {
        decoderState = DecoderState::READING_PAYLOAD;
        byteIndex = 0;
      }
      break;

    case DecoderState::READING_PAYLOAD:
      payloadBuffer[byteIndex++] = byte;
      if (byteIndex >= 9) {
        decoderState = DecoderState::READING_CRC;
        byteIndex = 0;
      }
      break;

    case DecoderState::READING_CRC:
      crcBuffer[byteIndex++] = byte;
      if (byteIndex >= 2) {
        // Complete packet received - verify CRC
        uint16_t receivedCrc = (crcBuffer[0] << 8) | crcBuffer[1];
        uint16_t calculatedCrc = calculateCRC16(payloadBuffer, 9);

        if (receivedCrc == calculatedCrc) {
          // CRC valid - dispatch based on command ID
          CommandId commandId = static_cast<CommandId>(payloadBuffer[0]);

          switch (commandId) {
            case CommandId::SERVO_COMMAND:
              handleServoCommand(payloadBuffer);
              break;

            default:
              // Unknown command ID - ignore
              break;
          }
        }
        // else: CRC mismatch - discard packet silently

        // Reset state machine for next packet
        decoderState = DecoderState::WAITING_FOR_START;
        byteIndex = 0;
      }
      break;
  }
}

/**
 * Serial reader task - monitors Serial for incoming data
 */
void vSerialReaderTask(void *pvParameters)
{
  while (1) {
    // Check if data is available on Serial
    if (Serial.available() > 0) {
      // Read one byte
      uint8_t receivedByte = Serial.read();

      // Call the stream decoder
      serialStreamDecoder(receivedByte);
    } else {
      // No data available, yield to other tasks
      // This is more responsive than vTaskDelay but doesn't busy-wait
      taskYIELD();
    }
  }
}

/**
 * Configure a single LEDC channel for servo control
 * @param channel LEDC channel number
 * @param gpio_num GPIO pin number
 */
void configureLedcChannel(ledc_channel_t channel, int gpio_num)
{
  ledc_channel_config_t ledc_channel = {
    .gpio_num = gpio_num,
    .speed_mode = LEDC_MODE,
    .channel = channel,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER,
    .duty = 0,
    .hpoint = 0,
    .flags = {.output_invert = 0}
  };
  ledc_channel_config(&ledc_channel);
}

/**
 * Set and update LEDC duty cycle for a channel
 * @param channel LEDC channel number
 * @param duty Duty cycle value (0-16383 for 14-bit resolution)
 */
void setLedcDuty(ledc_channel_t channel, uint32_t duty)
{
  ledc_set_duty(LEDC_MODE, channel, duty);
  ledc_update_duty(LEDC_MODE, channel);
}

/**
 * Blink task - toggles LED at regular intervals
 */
void vBlinkTask(void *pvParameters)
{
  while (1) {
    // Toggle LED (inverted logic)
    ledState = !ledState;
    digitalWrite(STATUS_LED, ledState ? LOW : HIGH);

    // Wait for next blink
    vTaskDelay(pdMS_TO_TICKS(BLINK_PERIOD_MS));
  }
}

void setup()
{
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);

  // Initialize status LED (GPIO8)
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, HIGH);  // LED OFF initially (inverted)

  // Wait for serial to be ready
  delay(1000);

  Serial.println("FreeRTOS Servo Controller Starting...");

  // we don´t use the servo library because the angular
  // resolution was limited to 10 bits or 2 degrees
  // of angular resolution.

  // Setup LEDC Timer
  Serial.println("Configuring LEDC timer...");

  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_MODE,
    .duty_resolution = LEDC_DUTY_RES,
    .timer_num = LEDC_TIMER,
    .freq_hz = LEDC_FREQUENCY,
    .clk_cfg = LEDC_USE_APB_CLK    // 80 MHz APB clock
  };

  esp_err_t err = ledc_timer_config(&ledc_timer);
  if (err != ESP_OK) {
    Serial.printf("LEDC timer config failed: %d\n", err);
  } else {
    Serial.println("LEDC timer configured successfully!");
  }

  // Setup LEDC Channels
  Serial.println("Configuring servo outputs...");

  configureLedcChannel(LEDC_CHANNEL_0, SERVO_PIN_0);
  configureLedcChannel(LEDC_CHANNEL_1, SERVO_PIN_1);
  configureLedcChannel(LEDC_CHANNEL_2, SERVO_PIN_2);
  configureLedcChannel(LEDC_CHANNEL_3, SERVO_PIN_3);

  // Initialize all servos to center position (π/2 radians = 90 degrees)
  uint32_t centerDuty = anglesToDutyCycle(SERVO_INITIAL_POSITION);

  setLedcDuty(LEDC_CHANNEL_0, centerDuty);
  setLedcDuty(LEDC_CHANNEL_1, centerDuty);
  setLedcDuty(LEDC_CHANNEL_2, centerDuty);
  setLedcDuty(LEDC_CHANNEL_3, centerDuty);

  Serial.println("Servo outputs configured successfully!");

  // Create queue for profiler commands (stream decoder -> profiler)
  profilerQueue = xQueueCreate(5, sizeof(ServoPositions));
  if (profilerQueue == NULL) {
    Serial.println("Failed to create profiler queue!");
  } else {
    Serial.println("Profiler queue created successfully!");
  }

  // Create queue for servo positions (profiler -> servo task)
  servoQueue = xQueueCreate(5, sizeof(ServoPositions));
  if (servoQueue == NULL) {
    Serial.println("Failed to create servo queue!");
  } else {
    Serial.println("Servo queue created successfully!");
  }

  // Create the profiler task
  xTaskCreate(vProfilerTask,       // Task function
              "Profiler",          // Task name
              2048,                // Stack size (bytes)
              NULL,                // Parameters
              3,                   // Priority (high - motion control)
              &profilerTaskHandle  // Task handle
  );
  Serial.println("Profiler task created!");

  // Create the servo control task
  xTaskCreate(vServoTask,       // Task function
              "ServoControl",   // Task name
              2048,             // Stack size (bytes)
              NULL,             // Parameters
              2,                // Priority (below profiler)
              &servoTaskHandle  // Task handle
  );
  Serial.println("Servo control task created!");

  // Create the serial reader task
  xTaskCreate(vSerialReaderTask,  // Task function
              "SerialReader",     // Task name
              2048,               // Stack size (bytes)
              NULL,               // Parameters
              0,                  // Priority (lowest - runs when others idle)
              &serialReaderTaskHandle  // Task handle
  );
  Serial.println("Serial reader task created!");

  // Create the blink task
  xTaskCreate(vBlinkTask,       // Task function
              "BlinkTask",      // Task name
              2048,             // Stack size (bytes)
              NULL,             // Parameters
              1,                // Priority
              &blinkTaskHandle  // Task handle
  );
  Serial.println("Blink task created!");
}

void loop()
{
  // Small delay to prevent watchdog issues
  vTaskDelay(pdMS_TO_TICKS(100));
}
