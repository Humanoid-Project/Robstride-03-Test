// Standalone driver for the WHEELTEC N100 IMU.
//
// A background thread owns the serial port and decodes frames into a latest
// sample snapshot. Control loops call latest() which never blocks on I/O.
#ifndef N100_IMU_DRIVER_HPP_
#define N100_IMU_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "n100/math.hpp"
#include "n100/protocol.hpp"
#include "n100/serial_port.hpp"

namespace n100 {

// One fused reading. Orientation and angular velocity come from the AHRS
// frame; acceleration, magnetometer and environment fields come from the most
// recent IMU frame, which is how the device splits its stream.
//
// "Body frame" below means the robot base frame when DriverConfig::mount_rotation
// is set, and the raw IMU case frame otherwise.
struct ImuSample {
  Quat orientation;                 // body -> reference frame
  Euler euler;                      // convenience decomposition of `orientation`
  Vec3 angular_velocity;            // rad/s, body frame, gyro bias removed
  // Same quantity taken straight from the IMU frame gyro instead of the AHRS
  // frame's fused rates, bias removed separately. The fused rates carry the
  // filter's phase lag, while policies trained in Isaac Lab assume a raw body
  // rate, so compare the two on hardware before choosing one. Only valid when
  // has_imu_frame is true.
  Vec3 angular_velocity_raw;        // rad/s, body frame, raw gyro bias removed
  Vec3 linear_acceleration;         // m/s^2, body frame
  Vec3 magnetic_field;              // Tesla, body frame, offsets removed
  Vec3 projected_gravity;           // unit vector, body frame, (0,0,-1) upright

  double imu_temperature = 0.0;     // degC
  double pressure = 0.0;            // Pa

  std::int64_t device_timestamp_us = 0;  // device clock, from the AHRS frame
  std::int64_t host_timestamp_ns = 0;    // steady_clock at decode time
  std::uint64_t seq = 0;                 // increments once per sample

  bool has_imu_frame = false;  // false until the first IMU frame is decoded
};

struct DriverStats {
  std::uint64_t imu_frames = 0;
  std::uint64_t ahrs_frames = 0;
  std::uint64_t insgps_frames = 0;
  std::uint64_t ground_frames = 0;
  std::uint64_t samples = 0;
  std::uint64_t crc8_errors = 0;
  std::uint64_t crc16_errors = 0;
  std::uint64_t frame_end_errors = 0;
  std::uint64_t dropped_bytes = 0;  // bytes discarded while resynchronising
  // Serial numbers the device used but that never arrived. Confirmed only once
  // 64 later frames have been seen, so out of order delivery is not miscounted
  // as loss; up to a window's worth may still be pending when the driver stops.
  std::uint64_t sn_lost = 0;
  std::uint64_t bytes_read = 0;
};

struct DriverConfig {
  std::string port = "/dev/ttyUSB0";
  int baudrate = 921600;

  // Ask the tty driver for minimal receive latency.
  bool low_latency = true;

  // The device reports orientation in its own reference frame. When true the
  // SDK premultiplies a fixed 180 degree rotation about the reference Y axis,
  // matching `device_type == 1` in the original ROS driver. This changes the
  // reference frame only, so body frame vectors (gyro, accel) are untouched.
  bool apply_reference_frame_transform = true;

  // Fixed rotation from the IMU case to the robot base frame, i.e. it maps a
  // vector expressed in the IMU frame to the same vector expressed in the base
  // frame. Every reported quantity is resolved in the base frame once this is
  // set, so a policy sees the robot's axes rather than the sensor's.
  //
  // Identity by default. To find it, run the `read_imu` example with the robot
  // upright: projected_gravity must read (0, 0, -1). If it reads (0, 0, +1) the
  // IMU is mounted inverted, so use Quat::fromAxisAngleX(M_PI).
  Quat mount_rotation;

  // Hard iron offsets in Tesla, subtracted after the mG -> T conversion.
  Vec3 magnetometer_offset{0.0, 0.0, 0.0};

  // Constant gyro bias in rad/s, subtracted from angular_velocity.
  // See calibrateGyroBias() to measure it at startup.
  Vec3 gyro_bias{0.0, 0.0, 0.0};

  // Reader thread poll timeout. Only affects how quickly stop() is noticed.
  int read_timeout_ms = 20;
};

class ImuDriver {
 public:
  explicit ImuDriver(DriverConfig config = {});
  ~ImuDriver();

  ImuDriver(const ImuDriver&) = delete;
  ImuDriver& operator=(const ImuDriver&) = delete;

  // Opens the port and starts the reader thread. Throws std::runtime_error if
  // the port cannot be opened.
  void start();

  // Stops the reader thread and closes the port. Safe to call twice.
  void stop();

  bool isRunning() const { return running_.load(std::memory_order_acquire); }

  // Copies the most recent sample. Returns false until the first one arrives.
  // Never blocks on the serial port.
  bool latest(ImuSample& out) const;

  // Blocks until a sample newer than `last_seq` is available. Pass 0 to accept
  // any sample. Returns false on timeout or if the driver stopped.
  bool waitForSample(ImuSample& out, std::chrono::milliseconds timeout,
                     std::uint64_t last_seq = 0) const;

  // Invoked on the reader thread for every decoded sample. Keep it short;
  // anything slow here stalls decoding. Set before start().
  void setSampleCallback(std::function<void(const ImuSample&)> callback);

  DriverStats stats() const;

  // Zeroes the counters. Opening the port mid-stream always costs a one time
  // resynchronisation, so calling this once the first samples are flowing makes
  // any later non-zero counter mean a real fault.
  void resetStats();

  // Averages angular velocity over `duration` and installs the result as the
  // gyro bias. The device must be completely still. Returns false if the
  // driver is not running or no samples arrived.
  bool calibrateGyroBias(std::chrono::milliseconds duration);

  // Bias of the AHRS fused rates, i.e. of ImuSample::angular_velocity.
  Vec3 gyroBias() const;
  void setGyroBias(const Vec3& bias);

  // Bias of the raw IMU frame gyro, i.e. of ImuSample::angular_velocity_raw.
  // calibrateGyroBias() measures both in the same pass.
  Vec3 gyroBiasRaw() const;
  void setGyroBiasRaw(const Vec3& bias);

  // Set when the reader thread aborted on an I/O error. Empty otherwise.
  std::string lastError() const;

  const DriverConfig& config() const { return config_; }

 private:
  // The reader thread accumulates counters into a local delta and merges it
  // under `stats_mutex_` once per read, so the hot path stays lock free.
  void readLoop();
  void parseBuffer(DriverStats& delta);
  void handleFrame(std::uint8_t type, std::uint8_t serial_num,
                   const std::uint8_t* payload, std::size_t length,
                   DriverStats& delta);
  void trackSerialNumber(std::uint8_t serial_num, DriverStats& delta);
  void publishSample(DriverStats& delta);
  void mergeStats(const DriverStats& delta);
  void setError(const std::string& message);

  DriverConfig config_;
  SerialPort port_;

  std::thread reader_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};

  // Reader thread state, no locking required.
  std::vector<std::uint8_t> buffer_;
  protocol::ImuPacket imu_packet_{};
  protocol::AhrsPacket ahrs_packet_{};
  bool has_imu_packet_ = false;
  // Serial number bookkeeping. The device shares one counter across frame
  // types but does not always transmit in counter order, so arrivals are
  // scored against a sliding window instead of a single expected value.
  bool has_serial_num_ = false;
  std::int64_t sn_extended_ = 0;      // highest serial number seen, unwrapped
  std::int64_t sn_window_base_ = 0;   // oldest serial number still scoreable
  std::uint64_t sn_window_mask_ = 0;  // one bit per serial number in the window
  Quat reference_transform_;
  Quat mount_rotation_;         // IMU frame -> base frame
  Quat mount_rotation_inverse_;
  bool has_mount_rotation_ = false;
  std::function<void(const ImuSample&)> callback_;

  mutable std::mutex sample_mutex_;
  mutable std::condition_variable sample_cv_;
  ImuSample latest_sample_;
  bool has_sample_ = false;

  mutable std::mutex stats_mutex_;
  DriverStats stats_;

  mutable std::mutex bias_mutex_;
  Vec3 gyro_bias_;
  Vec3 gyro_bias_raw_;

  mutable std::mutex error_mutex_;
  std::string last_error_;
};

}  // namespace n100

#endif  // N100_IMU_DRIVER_HPP_
