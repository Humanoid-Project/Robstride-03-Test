#include "n100/imu_driver.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

#include "n100/crc.hpp"

namespace n100 {
namespace {

constexpr double kPi = 3.141592653589793;

// mG -> Tesla
constexpr double kMilliGaussToTesla = 1.0e-7;

// Read chunk size. One AHRS frame is 56 bytes, so this covers several frames
// per syscall at 921600 baud without adding latency.
constexpr std::size_t kReadChunk = 512;

// Guard against unbounded growth when the stream is pure noise.
constexpr std::size_t kMaxBuffer = 8192;

// How far a serial number may arrive out of order before it counts as lost.
// Must not exceed 64, the width of the window bitmask.
constexpr std::int64_t kSnWindow = 64;

std::int64_t steadyNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

ImuDriver::ImuDriver(DriverConfig config) : config_(std::move(config)) {
  gyro_bias_ = config_.gyro_bias;

  // Equivalent to Rz(pi) * Rx(pi), the transform the original ROS driver
  // applied for device_type == 1. It reduces to a 180 degree turn about Y.
  reference_transform_ = config_.apply_reference_frame_transform
                             ? (Quat::fromAxisAngleZ(kPi) * Quat::fromAxisAngleX(kPi)).normalized()
                             : Quat();

  mount_rotation_ = config_.mount_rotation.normalized();
  mount_rotation_inverse_ = mount_rotation_.conjugate();
  has_mount_rotation_ = std::fabs(mount_rotation_.w) < 1.0 - 1e-12;

  buffer_.reserve(kMaxBuffer);
}

ImuDriver::~ImuDriver() { stop(); }

void ImuDriver::start() {
  if (running_.load(std::memory_order_acquire)) return;

  port_.open(config_.port, config_.baudrate, config_.low_latency);

  buffer_.clear();
  has_imu_packet_ = false;
  has_serial_num_ = false;
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    has_sample_ = false;
    latest_sample_ = ImuSample{};
  }

  stop_requested_.store(false, std::memory_order_release);
  running_.store(true, std::memory_order_release);
  reader_ = std::thread(&ImuDriver::readLoop, this);
}

void ImuDriver::stop() {
  stop_requested_.store(true, std::memory_order_release);
  if (reader_.joinable()) reader_.join();
  running_.store(false, std::memory_order_release);
  port_.close();
  sample_cv_.notify_all();
}

void ImuDriver::setSampleCallback(std::function<void(const ImuSample&)> callback) {
  callback_ = std::move(callback);
}

void ImuDriver::readLoop() {
  std::uint8_t chunk[kReadChunk];

  while (!stop_requested_.load(std::memory_order_acquire)) {
    DriverStats delta{};
    try {
      const std::size_t count = port_.read(chunk, sizeof(chunk), config_.read_timeout_ms);
      if (count > 0) {
        delta.bytes_read += count;
        buffer_.insert(buffer_.end(), chunk, chunk + count);
        if (buffer_.size() > kMaxBuffer) {
          const std::size_t excess = buffer_.size() - kMaxBuffer;
          buffer_.erase(buffer_.begin(), buffer_.begin() + excess);
          delta.dropped_bytes += excess;
        }
        parseBuffer(delta);
      }
    } catch (const std::exception& e) {
      setError(e.what());
      break;
    }
    mergeStats(delta);
  }

  running_.store(false, std::memory_order_release);
  sample_cv_.notify_all();
}

void ImuDriver::parseBuffer(DriverStats& delta) {
  std::size_t pos = 0;
  const std::size_t size = buffer_.size();

  while (true) {
    // Resynchronise on the frame head.
    while (pos < size && buffer_[pos] != protocol::kFrameHead) {
      ++pos;
      ++delta.dropped_bytes;
    }
    if (size - pos < protocol::kHeaderSize) break;

    const std::uint8_t* header = buffer_.data() + pos;
    const std::uint8_t type = header[1];
    const std::uint8_t length = header[2];
    const std::uint8_t serial_num = header[3];

    if (!protocol::isKnownType(type) || !protocol::lengthMatchesType(type, length)) {
      ++pos;
      ++delta.dropped_bytes;
      continue;
    }

    if (crc8(header, protocol::kCrc8Span) != header[4]) {
      ++delta.crc8_errors;
      ++pos;
      ++delta.dropped_bytes;
      continue;
    }

    const std::size_t frame_size = protocol::kHeaderSize + length + 1;
    if (size - pos < frame_size) break;  // wait for the rest of the frame

    // Ground frames carry an undocumented payload; skip them without decoding,
    // but still consume the serial number so loss tracking stays aligned.
    if (protocol::isGroundType(type)) {
      trackSerialNumber(serial_num, delta);
      ++delta.ground_frames;
      pos += frame_size;
      continue;
    }

    const std::uint8_t* payload = header + protocol::kHeaderSize;
    const std::uint16_t expected_crc16 =
        static_cast<std::uint16_t>((static_cast<std::uint16_t>(header[5]) << 8) | header[6]);
    if (crc16(payload, length) != expected_crc16) {
      ++delta.crc16_errors;
      ++pos;
      ++delta.dropped_bytes;
      continue;
    }

    if (payload[length] != protocol::kFrameEnd) {
      ++delta.frame_end_errors;
      ++pos;
      ++delta.dropped_bytes;
      continue;
    }

    handleFrame(type, serial_num, payload, length, delta);
    pos += frame_size;
  }

  buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
}

void ImuDriver::handleFrame(std::uint8_t type, std::uint8_t serial_num,
                            const std::uint8_t* payload, std::size_t length,
                            DriverStats& delta) {
  trackSerialNumber(serial_num, delta);

  switch (type) {
    case protocol::kTypeImu:
      std::memcpy(&imu_packet_, payload, sizeof(imu_packet_));
      has_imu_packet_ = true;
      ++delta.imu_frames;
      break;

    case protocol::kTypeAhrs:
      std::memcpy(&ahrs_packet_, payload, sizeof(ahrs_packet_));
      ++delta.ahrs_frames;
      // The AHRS frame carries orientation, so it drives the sample rate.
      publishSample(delta);
      break;

    case protocol::kTypeInsGps:
      ++delta.insgps_frames;
      break;

    default:
      break;
  }
  (void)length;
}

// The device shares one 8 bit counter across frame types, but the once per
// second ground frame is transmitted after the frame that follows it in
// counter order. Scoring each arrival against the previous one therefore
// reports a whole counter wrap of phantom losses per ground frame. Instead
// every serial number is recorded in a sliding window and only counted as lost
// once it has fallen out of that window without ever arriving.
void ImuDriver::trackSerialNumber(std::uint8_t serial_num, DriverStats& delta) {
  if (!has_serial_num_) {
    has_serial_num_ = true;
    sn_extended_ = serial_num;
    sn_window_base_ = serial_num;
    sn_window_mask_ = 1;
    return;
  }

  // Unwrap the counter relative to the highest value seen. A signed step keeps
  // reordering (a small backwards move) distinct from advancing.
  const std::int8_t step = static_cast<std::int8_t>(
      serial_num - static_cast<std::uint8_t>(sn_extended_ & 0xFF));
  const std::int64_t arrived = sn_extended_ + step;
  if (arrived > sn_extended_) sn_extended_ = arrived;

  // Older than the window: already scored, nothing left to decide.
  if (arrived < sn_window_base_) return;

  while (arrived >= sn_window_base_ + kSnWindow) {
    if ((sn_window_mask_ & 1ull) == 0) ++delta.sn_lost;
    sn_window_mask_ >>= 1;
    ++sn_window_base_;
  }
  sn_window_mask_ |= 1ull << (arrived - sn_window_base_);
}

void ImuDriver::publishSample(DriverStats& delta) {
  ImuSample sample;

  Quat orientation(ahrs_packet_.qw, ahrs_packet_.qx, ahrs_packet_.qy, ahrs_packet_.qz);
  orientation = orientation.normalized();
  if (config_.apply_reference_frame_transform) {
    orientation = (reference_transform_ * orientation).normalized();
  }
  // Re-express the rotation as base -> reference instead of IMU -> reference.
  if (has_mount_rotation_) {
    orientation = (orientation * mount_rotation_inverse_).normalized();
  }
  sample.orientation = orientation;
  sample.euler = toEuler(orientation);
  sample.projected_gravity = projectedGravity(orientation);

  Vec3 bias;
  Vec3 bias_raw;
  {
    std::lock_guard<std::mutex> lock(bias_mutex_);
    bias = gyro_bias_;
    bias_raw = gyro_bias_raw_;
  }
  // The reference frame transform does not affect body frame rates, but the
  // mount rotation does. Bias is removed last so it lives in the same frame as
  // the samples calibrateGyroBias() averaged.
  Vec3 angular_velocity{ahrs_packet_.roll_speed,
                        ahrs_packet_.pitch_speed,
                        ahrs_packet_.heading_speed};
  if (has_mount_rotation_) angular_velocity = mount_rotation_.rotate(angular_velocity);
  sample.angular_velocity = angular_velocity - bias;

  sample.has_imu_frame = has_imu_packet_;
  if (has_imu_packet_) {
    Vec3 acceleration{imu_packet_.accelerometer_x,
                      imu_packet_.accelerometer_y,
                      imu_packet_.accelerometer_z};
    Vec3 magnetic_field = Vec3{imu_packet_.magnetometer_x * kMilliGaussToTesla,
                               imu_packet_.magnetometer_y * kMilliGaussToTesla,
                               imu_packet_.magnetometer_z * kMilliGaussToTesla} -
                          config_.magnetometer_offset;
    Vec3 gyroscope{imu_packet_.gyroscope_x,
                   imu_packet_.gyroscope_y,
                   imu_packet_.gyroscope_z};
    if (has_mount_rotation_) {
      acceleration = mount_rotation_.rotate(acceleration);
      magnetic_field = mount_rotation_.rotate(magnetic_field);
      gyroscope = mount_rotation_.rotate(gyroscope);
    }
    sample.linear_acceleration = acceleration;
    sample.magnetic_field = magnetic_field;
    sample.angular_velocity_raw = gyroscope - bias_raw;
    sample.imu_temperature = imu_packet_.imu_temperature;
    sample.pressure = imu_packet_.pressure;
  }

  sample.device_timestamp_us = ahrs_packet_.timestamp;
  sample.host_timestamp_ns = steadyNowNs();

  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    sample.seq = latest_sample_.seq + 1;
    latest_sample_ = sample;
    has_sample_ = true;
  }
  sample_cv_.notify_all();
  ++delta.samples;

  if (callback_) callback_(sample);
}

bool ImuDriver::latest(ImuSample& out) const {
  std::lock_guard<std::mutex> lock(sample_mutex_);
  if (!has_sample_) return false;
  out = latest_sample_;
  return true;
}

bool ImuDriver::waitForSample(ImuSample& out, std::chrono::milliseconds timeout,
                              std::uint64_t last_seq) const {
  std::unique_lock<std::mutex> lock(sample_mutex_);
  const bool ready = sample_cv_.wait_for(lock, timeout, [&] {
    return (has_sample_ && latest_sample_.seq > last_seq) ||
           !running_.load(std::memory_order_acquire);
  });
  if (!ready || !has_sample_ || latest_sample_.seq <= last_seq) return false;
  out = latest_sample_;
  return true;
}

DriverStats ImuDriver::stats() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

void ImuDriver::resetStats() {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_ = DriverStats{};
}

void ImuDriver::mergeStats(const DriverStats& delta) {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.imu_frames += delta.imu_frames;
  stats_.ahrs_frames += delta.ahrs_frames;
  stats_.insgps_frames += delta.insgps_frames;
  stats_.ground_frames += delta.ground_frames;
  stats_.samples += delta.samples;
  stats_.crc8_errors += delta.crc8_errors;
  stats_.crc16_errors += delta.crc16_errors;
  stats_.frame_end_errors += delta.frame_end_errors;
  stats_.dropped_bytes += delta.dropped_bytes;
  stats_.sn_lost += delta.sn_lost;
  stats_.bytes_read += delta.bytes_read;
}

bool ImuDriver::calibrateGyroBias(std::chrono::milliseconds duration) {
  if (!isRunning()) return false;

  // Measure against the uncorrected signals.
  setGyroBias(Vec3{0.0, 0.0, 0.0});
  setGyroBiasRaw(Vec3{0.0, 0.0, 0.0});

  const auto deadline = std::chrono::steady_clock::now() + duration;
  Vec3 accumulator;
  Vec3 accumulator_raw;
  std::uint64_t count = 0;
  std::uint64_t count_raw = 0;
  std::uint64_t last_seq = 0;

  ImuSample sample;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!waitForSample(sample, std::chrono::milliseconds(100), last_seq)) {
      if (!isRunning()) break;
      continue;
    }
    last_seq = sample.seq;
    accumulator += sample.angular_velocity;
    ++count;
    if (sample.has_imu_frame) {
      accumulator_raw += sample.angular_velocity_raw;
      ++count_raw;
    }
  }

  if (count == 0) return false;
  setGyroBias(accumulator * (1.0 / static_cast<double>(count)));
  if (count_raw > 0) {
    setGyroBiasRaw(accumulator_raw * (1.0 / static_cast<double>(count_raw)));
  }
  return true;
}

Vec3 ImuDriver::gyroBias() const {
  std::lock_guard<std::mutex> lock(bias_mutex_);
  return gyro_bias_;
}

void ImuDriver::setGyroBias(const Vec3& bias) {
  std::lock_guard<std::mutex> lock(bias_mutex_);
  gyro_bias_ = bias;
}

Vec3 ImuDriver::gyroBiasRaw() const {
  std::lock_guard<std::mutex> lock(bias_mutex_);
  return gyro_bias_raw_;
}

void ImuDriver::setGyroBiasRaw(const Vec3& bias) {
  std::lock_guard<std::mutex> lock(bias_mutex_);
  gyro_bias_raw_ = bias;
}

std::string ImuDriver::lastError() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

void ImuDriver::setError(const std::string& message) {
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_ = message;
}

}  // namespace n100
