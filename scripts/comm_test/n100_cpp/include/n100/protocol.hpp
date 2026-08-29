// FDILink binary protocol used by the WHEELTEC N100 IMU.
//
// Frame layout (little endian payload):
//   [0] 0xFC          frame head
//   [1] data type     0x40 IMU / 0x41 AHRS / 0x42 INSGPS / 0xF0 ground
//   [2] data size     payload length in bytes
//   [3] serial number incremented by the device, used for loss detection
//   [4] CRC8          over bytes [0..3]
//   [5] CRC16 high    over the payload
//   [6] CRC16 low
//   [7 .. 7+size-1]   payload
//   [7+size]          0xFD frame end
#ifndef N100_PROTOCOL_HPP_
#define N100_PROTOCOL_HPP_

#include <cstdint>
#include <cstddef>

namespace n100 {
namespace protocol {

inline constexpr std::uint8_t kFrameHead = 0xFC;
inline constexpr std::uint8_t kFrameEnd = 0xFD;

inline constexpr std::uint8_t kTypeImu = 0x40;
inline constexpr std::uint8_t kTypeAhrs = 0x41;
inline constexpr std::uint8_t kTypeInsGps = 0x42;
inline constexpr std::uint8_t kTypeGround = 0xF0;
// The device also emits 0x50 frames that carry no documented payload.
inline constexpr std::uint8_t kTypeGroundAlt = 0x50;

inline constexpr std::uint8_t kImuLen = 0x38;     // 56
inline constexpr std::uint8_t kAhrsLen = 0x30;    // 48
inline constexpr std::uint8_t kInsGpsLen = 0x54;  // 84

inline constexpr std::size_t kHeaderSize = 7;
inline constexpr std::size_t kCrc8Span = 4;  // head, type, size, serial number

#pragma pack(push, 1)

struct ImuPacket {
  float gyroscope_x;           // rad/s
  float gyroscope_y;           // rad/s
  float gyroscope_z;           // rad/s
  float accelerometer_x;       // m/s^2
  float accelerometer_y;       // m/s^2
  float accelerometer_z;       // m/s^2
  float magnetometer_x;        // mG
  float magnetometer_y;        // mG
  float magnetometer_z;        // mG
  float imu_temperature;       // degC
  float pressure;              // Pa
  float pressure_temperature;  // degC
  std::int64_t timestamp;      // us, device clock
};

struct AhrsPacket {
  float roll_speed;         // rad/s
  float pitch_speed;        // rad/s
  float heading_speed;      // rad/s
  float roll;               // rad
  float pitch;              // rad
  float heading;            // rad
  float qw;                 // quaternion, device frame
  float qx;
  float qy;
  float qz;
  std::int64_t timestamp;   // us, device clock
};

struct InsGpsPacket {
  float body_velocity_x;
  float body_velocity_y;
  float body_velocity_z;
  float body_acceleration_x;
  float body_acceleration_y;
  float body_acceleration_z;
  double location_north;
  double location_east;
  double location_down;
  float velocity_north;
  float velocity_east;
  float velocity_down;
  float acceleration_north;
  float acceleration_east;
  float acceleration_down;
  float pressure_altitude;
  std::int64_t timestamp;
};

#pragma pack(pop)

static_assert(sizeof(ImuPacket) == kImuLen, "ImuPacket must match the wire layout");
static_assert(sizeof(AhrsPacket) == kAhrsLen, "AhrsPacket must match the wire layout");
static_assert(sizeof(InsGpsPacket) == kInsGpsLen, "InsGpsPacket must match the wire layout");

// True for every frame type the parser recognises.
inline bool isKnownType(std::uint8_t type) {
  return type == kTypeImu || type == kTypeAhrs || type == kTypeInsGps ||
         type == kTypeGround || type == kTypeGroundAlt;
}

// Ground frames carry an undocumented payload, so they are skipped rather than decoded.
inline bool isGroundType(std::uint8_t type) {
  return type == kTypeGround || type == kTypeGroundAlt;
}

// Payload length is fixed for the decoded frame types.
inline bool lengthMatchesType(std::uint8_t type, std::uint8_t size) {
  switch (type) {
    case kTypeImu: return size == kImuLen;
    case kTypeAhrs: return size == kAhrsLen;
    case kTypeInsGps: return size == kInsGpsLen;
    default: return true;
  }
}

}  // namespace protocol
}  // namespace n100

#endif  // N100_PROTOCOL_HPP_
