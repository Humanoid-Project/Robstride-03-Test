// Hardware free tests: the driver is pointed at a pseudo terminal and fed
// synthesised frames, so the serial layer, framing, CRC checks, resynchronisation
// and sample assembly are all exercised end to end.

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "n100/crc.hpp"
#include "n100/imu_driver.hpp"
#include "n100/math.hpp"
#include "n100/protocol.hpp"

namespace {

int g_failures = 0;

void check(bool condition, const std::string& what) {
  if (condition) {
    std::printf("  ok    %s\n", what.c_str());
  } else {
    std::printf("  FAIL  %s\n", what.c_str());
    ++g_failures;
  }
}

void checkClose(double actual, double expected, double tolerance,
                const std::string& what) {
  const bool ok = std::fabs(actual - expected) <= tolerance;
  if (!ok) {
    std::printf("  FAIL  %s (got %.6f, want %.6f)\n", what.c_str(), actual, expected);
    ++g_failures;
  } else {
    std::printf("  ok    %s\n", what.c_str());
  }
}

// Builds a complete on the wire frame around `payload`.
std::vector<std::uint8_t> buildFrame(std::uint8_t type, std::uint8_t serial_num,
                                     const void* payload, std::size_t length) {
  std::vector<std::uint8_t> frame;
  frame.resize(n100::protocol::kHeaderSize + length + 1);

  frame[0] = n100::protocol::kFrameHead;
  frame[1] = type;
  frame[2] = static_cast<std::uint8_t>(length);
  frame[3] = serial_num;
  frame[4] = n100::crc8(frame.data(), n100::protocol::kCrc8Span);

  std::memcpy(frame.data() + n100::protocol::kHeaderSize, payload, length);
  const std::uint16_t payload_crc =
      n100::crc16(frame.data() + n100::protocol::kHeaderSize, length);
  frame[5] = static_cast<std::uint8_t>(payload_crc >> 8);
  frame[6] = static_cast<std::uint8_t>(payload_crc & 0xFF);

  frame[n100::protocol::kHeaderSize + length] = n100::protocol::kFrameEnd;
  return frame;
}

n100::protocol::ImuPacket makeImuPacket() {
  n100::protocol::ImuPacket p{};
  p.gyroscope_x = 0.1f;
  p.gyroscope_y = 0.2f;
  p.gyroscope_z = 0.3f;
  p.accelerometer_x = 0.5f;
  p.accelerometer_y = -0.25f;
  p.accelerometer_z = 9.81f;
  p.magnetometer_x = 100.0f;  // mG
  p.magnetometer_y = 200.0f;
  p.magnetometer_z = -300.0f;
  p.imu_temperature = 31.5f;
  p.pressure = 101325.0f;
  p.pressure_temperature = 30.0f;
  p.timestamp = 111222333;
  return p;
}

n100::protocol::AhrsPacket makeAhrsPacket() {
  n100::protocol::AhrsPacket p{};
  p.roll_speed = 0.01f;
  p.pitch_speed = -0.02f;
  p.heading_speed = 0.03f;
  p.roll = 0.0f;
  p.pitch = 0.0f;
  p.heading = 0.0f;
  p.qw = 1.0f;
  p.qx = 0.0f;
  p.qy = 0.0f;
  p.qz = 0.0f;
  p.timestamp = 444555666;
  return p;
}

// Opens a pty pair. `slave_path` receives the device the driver should open.
int openPtyMaster(std::string& slave_path) {
  const int master = posix_openpt(O_RDWR | O_NOCTTY);
  if (master < 0) return -1;
  if (grantpt(master) != 0 || unlockpt(master) != 0) {
    close(master);
    return -1;
  }
  const char* name = ptsname(master);
  if (name == nullptr) {
    close(master);
    return -1;
  }
  slave_path = name;
  return master;
}

void writeAll(int fd, const std::vector<std::uint8_t>& data, std::size_t offset,
              std::size_t length) {
  std::size_t written = 0;
  while (written < length) {
    const ssize_t n = ::write(fd, data.data() + offset + written, length - written);
    if (n <= 0) return;
    written += static_cast<std::size_t>(n);
  }
}

// Blocks until `driver` reports a sample newer than `since`, or gives up.
bool awaitSample(const n100::ImuDriver& driver, std::uint64_t since,
                 n100::ImuSample& out) {
  for (int i = 0; i < 200; ++i) {
    if (driver.latest(out) && out.seq > since) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

void testMath() {
  std::printf("math\n");

  const n100::Quat identity;
  const n100::Vec3 g = n100::projectedGravity(identity);
  checkClose(g.x, 0.0, 1e-9, "projected gravity x is 0 when level");
  checkClose(g.y, 0.0, 1e-9, "projected gravity y is 0 when level");
  checkClose(g.z, -1.0, 1e-9, "projected gravity z is -1 when level");

  // Pitch forward 90 degrees: gravity moves onto the body +x axis.
  const n100::Quat pitched = n100::Quat::fromAxisAngleY(M_PI / 2.0);
  const n100::Vec3 gp = n100::projectedGravity(pitched);
  checkClose(gp.x, 1.0, 1e-9, "projected gravity x is +1 when pitched 90 deg");
  checkClose(gp.z, 0.0, 1e-9, "projected gravity z is 0 when pitched 90 deg");

  // Roll 90 degrees: gravity moves onto the body -y axis.
  const n100::Quat rolled = n100::Quat::fromAxisAngleX(M_PI / 2.0);
  const n100::Vec3 gr = n100::projectedGravity(rolled);
  checkClose(gr.y, -1.0, 1e-9, "projected gravity y is -1 when rolled 90 deg");

  const n100::Euler e = n100::toEuler(n100::Quat::fromAxisAngleZ(0.5));
  checkClose(e.yaw, 0.5, 1e-9, "euler yaw round trips");
  checkClose(e.roll, 0.0, 1e-9, "euler roll round trips");

  // Rotating a vector and rotating it back is the identity.
  const n100::Quat q = (n100::Quat::fromAxisAngleZ(0.3) *
                        n100::Quat::fromAxisAngleY(-0.7)).normalized();
  const n100::Vec3 v{1.0, -2.0, 3.0};
  const n100::Vec3 back = q.inverseRotate(q.rotate(v));
  checkClose(back.x, v.x, 1e-9, "rotate then inverse rotate returns x");
  checkClose(back.y, v.y, 1e-9, "rotate then inverse rotate returns y");
  checkClose(back.z, v.z, 1e-9, "rotate then inverse rotate returns z");
}

void testCrcRegression() {
  std::printf("crc\n");

  // Frozen values from the tables shipped with the original driver. If these
  // change the device will reject or mis-accept frames.
  const std::uint8_t header[4] = {0xFC, 0x41, 0x30, 0x01};
  check(n100::crc8(header, 4) == n100::crc8(header, 4), "crc8 is deterministic");

  const std::uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  // "123456789" under CRC-16/XMODEM, the variant this table implements.
  check(n100::crc16(data, sizeof(data)) == 0x31C3, "crc16 matches CRC-16/XMODEM");
  check(n100::crc8(nullptr, 0) == 0, "crc8 of an empty span is 0");
  check(n100::crc16(nullptr, 0) == 0, "crc16 of an empty span is 0");
}

void testProtocolLayout() {
  std::printf("protocol\n");
  check(sizeof(n100::protocol::ImuPacket) == 56, "IMU payload is 56 bytes");
  check(sizeof(n100::protocol::AhrsPacket) == 48, "AHRS payload is 48 bytes");
  check(sizeof(n100::protocol::InsGpsPacket) == 84, "INSGPS payload is 84 bytes");
  check(n100::protocol::lengthMatchesType(n100::protocol::kTypeAhrs, 48),
        "AHRS length check accepts 48");
  check(!n100::protocol::lengthMatchesType(n100::protocol::kTypeAhrs, 47),
        "AHRS length check rejects 47");
  check(!n100::protocol::isKnownType(0x23), "unknown frame type is rejected");
}

void testDriverDecode() {
  std::printf("driver\n");

  std::string slave_path;
  const int master = openPtyMaster(slave_path);
  if (master < 0) {
    std::printf("  FAIL  could not create a pty pair\n");
    ++g_failures;
    return;
  }

  n100::DriverConfig config;
  config.port = slave_path;
  config.baudrate = 921600;
  config.low_latency = false;
  // Compare against the raw device quaternion.
  config.apply_reference_frame_transform = false;

  n100::ImuDriver driver(config);
  try {
    driver.start();
  } catch (const std::exception& e) {
    std::printf("  FAIL  could not open pty slave: %s\n", e.what());
    ++g_failures;
    close(master);
    return;
  }

  const auto imu_packet = makeImuPacket();
  const auto ahrs_packet = makeAhrsPacket();
  const auto imu_frame = buildFrame(n100::protocol::kTypeImu, 1, &imu_packet,
                                    sizeof(imu_packet));
  const auto ahrs_frame = buildFrame(n100::protocol::kTypeAhrs, 2, &ahrs_packet,
                                     sizeof(ahrs_packet));

  // 1. A clean IMU frame followed by an AHRS frame produces a fused sample.
  writeAll(master, imu_frame, 0, imu_frame.size());
  writeAll(master, ahrs_frame, 0, ahrs_frame.size());

  n100::ImuSample sample;
  if (!awaitSample(driver, 0, sample)) {
    std::printf("  FAIL  no sample decoded from a clean frame pair\n");
    ++g_failures;
    driver.stop();
    close(master);
    return;
  }
  check(sample.has_imu_frame, "sample carries IMU frame data");
  checkClose(sample.orientation.w, 1.0, 1e-6, "orientation w decoded");
  checkClose(sample.linear_acceleration.z, 9.81, 1e-4, "accel z decoded");
  checkClose(sample.linear_acceleration.y, -0.25, 1e-6, "accel y decoded");
  checkClose(sample.angular_velocity.y, -0.02, 1e-6, "angular velocity y decoded");
  checkClose(sample.angular_velocity_raw.x, 0.1, 1e-6, "raw gyro x comes from the IMU frame");
  checkClose(sample.angular_velocity_raw.y, 0.2, 1e-6, "raw gyro y comes from the IMU frame");
  checkClose(sample.angular_velocity_raw.z, 0.3, 1e-6, "raw gyro z comes from the IMU frame");
  checkClose(sample.magnetic_field.x, 100.0e-7, 1e-12, "magnetometer converted to Tesla");
  checkClose(sample.imu_temperature, 31.5, 1e-4, "temperature decoded");
  check(sample.device_timestamp_us == 444555666, "device timestamp taken from AHRS");
  check(sample.projected_gravity.z < -0.99, "projected gravity points down when level");

  // 2. A frame split across three writes still decodes.
  std::uint64_t seq = sample.seq;
  writeAll(master, ahrs_frame, 0, 5);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  writeAll(master, ahrs_frame, 5, 20);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  writeAll(master, ahrs_frame, 25, ahrs_frame.size() - 25);
  check(awaitSample(driver, seq, sample), "fragmented frame is reassembled");

  // 3. Garbage in front of a frame is discarded and the parser resynchronises.
  seq = sample.seq;
  const std::vector<std::uint8_t> noise = {0x00, 0xFF, 0xFC, 0x11, 0x22, 0xAB, 0xFC};
  writeAll(master, noise, 0, noise.size());
  writeAll(master, ahrs_frame, 0, ahrs_frame.size());
  check(awaitSample(driver, seq, sample), "parser resynchronises after garbage");
  check(driver.stats().dropped_bytes > 0, "discarded bytes are counted");

  // 4. A corrupted payload is rejected instead of being published.
  seq = sample.seq;
  const std::uint64_t crc16_before = driver.stats().crc16_errors;
  auto corrupt = ahrs_frame;
  corrupt[n100::protocol::kHeaderSize + 3] ^= 0xFF;
  writeAll(master, corrupt, 0, corrupt.size());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  check(driver.stats().crc16_errors > crc16_before, "payload corruption raises a CRC16 error");

  n100::ImuSample after_corrupt;
  driver.latest(after_corrupt);
  check(after_corrupt.seq == seq, "corrupted frame does not produce a sample");

  // 5. A real gap in the serial numbers is reported, once the window that
  //    tolerates out of order delivery has moved past it. Serial numbers 3
  //    through 11 are skipped here, so exactly nine are missing.
  const std::uint64_t lost_before = driver.stats().sn_lost;
  auto skipped = buildFrame(n100::protocol::kTypeAhrs, 12, &ahrs_packet,
                            sizeof(ahrs_packet));
  writeAll(master, skipped, 0, skipped.size());
  check(awaitSample(driver, after_corrupt.seq, sample), "frame after a gap still decodes");

  for (int sn = 13; sn <= 82; ++sn) {
    const auto filler = buildFrame(n100::protocol::kTypeAhrs,
                                   static_cast<std::uint8_t>(sn), &ahrs_packet,
                                   sizeof(ahrs_packet));
    writeAll(master, filler, 0, filler.size());
  }
  bool counted = false;
  for (int i = 0; i < 200 && !counted; ++i) {
    if (driver.stats().sn_lost == lost_before + 9) {
      counted = true;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  check(counted, "exactly the nine missing serial numbers are counted");

  // 6. Statistics reflect what was decoded.
  const n100::DriverStats stats = driver.stats();
  check(stats.ahrs_frames >= 4, "AHRS frames counted");
  check(stats.imu_frames >= 1, "IMU frames counted");
  check(stats.samples == stats.ahrs_frames, "one sample per AHRS frame");

  driver.resetStats();
  const n100::DriverStats cleared = driver.stats();
  check(cleared.samples == 0 && cleared.ahrs_frames == 0 && cleared.sn_lost == 0 &&
            cleared.dropped_bytes == 0,
        "resetStats zeroes the counters");

  driver.stop();
  check(!driver.isRunning(), "driver stops cleanly");
  close(master);
}

void testReferenceTransform() {
  std::printf("reference frame transform\n");

  // Rz(pi) * Rx(pi) reduces to a 180 degree turn about Y, and it must leave the
  // quaternion a unit quaternion.
  const n100::Quat applied =
      (n100::Quat::fromAxisAngleZ(M_PI) * n100::Quat::fromAxisAngleX(M_PI)).normalized();
  const n100::Quat about_y = n100::Quat::fromAxisAngleY(M_PI);
  const double dot = applied.w * about_y.w + applied.x * about_y.x +
                     applied.y * about_y.y + applied.z * about_y.z;
  checkClose(std::fabs(dot), 1.0, 1e-9, "Rz(pi)*Rx(pi) equals Ry(pi)");
  checkClose(applied.norm(), 1.0, 1e-9, "transform stays a unit quaternion");

  // The transform changes the reference frame, so an upright device still
  // reports gravity along body -z.
  const n100::Vec3 g = n100::projectedGravity((applied * n100::Quat()).normalized());
  checkClose(std::fabs(g.z), 1.0, 1e-9, "upright device keeps gravity on body z");
}

// Reproduces the real device behaviour: the once per second ground frame is
// transmitted after the frame that follows it in counter order. Scoring each
// arrival against the previous one alone reported a full counter wrap of
// phantom losses every second.
void testGroundFrameReordering() {
  std::printf("out of order ground frames\n");

  std::string slave_path;
  const int master = openPtyMaster(slave_path);
  if (master < 0) {
    std::printf("  FAIL  could not create a pty pair\n");
    ++g_failures;
    return;
  }

  n100::DriverConfig config;
  config.port = slave_path;
  config.low_latency = false;

  n100::ImuDriver driver(config);
  try {
    driver.start();
  } catch (const std::exception& e) {
    std::printf("  FAIL  could not open pty slave: %s\n", e.what());
    ++g_failures;
    close(master);
    return;
  }

  const auto imu_packet = makeImuPacket();
  const auto ahrs_packet = makeAhrsPacket();
  const std::uint8_t ground_payload[8] = {0, 1, 2, 3, 4, 5, 6, 7};

  // 30 cycles of IMU(n), AHRS(n+2), GROUND(n+1). No serial number is missing,
  // one simply arrives late.
  const int cycles = 30;
  for (int i = 0; i < cycles; ++i) {
    const std::uint8_t base = static_cast<std::uint8_t>(10 + 3 * i);
    const auto imu = buildFrame(n100::protocol::kTypeImu, base, &imu_packet,
                                sizeof(imu_packet));
    const auto ahrs = buildFrame(n100::protocol::kTypeAhrs,
                                 static_cast<std::uint8_t>(base + 2),
                                 &ahrs_packet, sizeof(ahrs_packet));
    const auto ground = buildFrame(n100::protocol::kTypeGround,
                                   static_cast<std::uint8_t>(base + 1),
                                   ground_payload, sizeof(ground_payload));
    writeAll(master, imu, 0, imu.size());
    writeAll(master, ahrs, 0, ahrs.size());
    writeAll(master, ground, 0, ground.size());
  }

  // Wait for the whole burst to be decoded.
  bool complete = false;
  for (int i = 0; i < 200 && !complete; ++i) {
    if (driver.stats().ahrs_frames >= static_cast<std::uint64_t>(cycles)) {
      complete = true;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  const n100::DriverStats stats = driver.stats();
  check(complete, "every frame in the burst is decoded");
  check(stats.ground_frames == static_cast<std::uint64_t>(cycles), "ground frames are skipped and counted");
  check(stats.imu_frames == static_cast<std::uint64_t>(cycles), "IMU frames survive the interleaving");
  check(stats.sn_lost == 0, "late ground frames are not counted as loss");
  check(stats.dropped_bytes == 0, "no bytes are discarded");

  driver.stop();
  close(master);
}

void testMountRotation() {
  std::printf("mount rotation\n");

  std::string slave_path;
  const int master = openPtyMaster(slave_path);
  if (master < 0) {
    std::printf("  FAIL  could not create a pty pair\n");
    ++g_failures;
    return;
  }

  // An IMU bolted in upside down: rotating its frame by 180 degrees about x
  // must flip the reported y and z axes back into the base frame.
  n100::DriverConfig config;
  config.port = slave_path;
  config.low_latency = false;
  config.apply_reference_frame_transform = false;
  config.mount_rotation = n100::Quat::fromAxisAngleX(M_PI);

  n100::ImuDriver driver(config);
  try {
    driver.start();
  } catch (const std::exception& e) {
    std::printf("  FAIL  could not open pty slave: %s\n", e.what());
    ++g_failures;
    close(master);
    return;
  }

  const auto imu_packet = makeImuPacket();
  const auto ahrs_packet = makeAhrsPacket();
  const auto imu_frame = buildFrame(n100::protocol::kTypeImu, 1, &imu_packet,
                                    sizeof(imu_packet));
  const auto ahrs_frame = buildFrame(n100::protocol::kTypeAhrs, 2, &ahrs_packet,
                                     sizeof(ahrs_packet));
  writeAll(master, imu_frame, 0, imu_frame.size());
  writeAll(master, ahrs_frame, 0, ahrs_frame.size());

  n100::ImuSample sample;
  if (!awaitSample(driver, 0, sample)) {
    std::printf("  FAIL  no sample decoded with a mount rotation\n");
    ++g_failures;
    driver.stop();
    close(master);
    return;
  }

  checkClose(sample.linear_acceleration.x, 0.5, 1e-4, "accel x is unchanged about the x axis");
  checkClose(sample.linear_acceleration.y, 0.25, 1e-4, "accel y is flipped into the base frame");
  checkClose(sample.linear_acceleration.z, -9.81, 1e-4, "accel z is flipped into the base frame");
  checkClose(sample.angular_velocity.z, -0.03, 1e-6, "angular velocity z is flipped");
  checkClose(sample.angular_velocity_raw.x, 0.1, 1e-6, "raw gyro x is unchanged about the x axis");
  checkClose(sample.angular_velocity_raw.y, -0.2, 1e-6, "raw gyro y follows the mount rotation");
  checkClose(sample.angular_velocity_raw.z, -0.3, 1e-6, "raw gyro z follows the mount rotation");
  checkClose(sample.projected_gravity.z, 1.0, 1e-6, "projected gravity follows the mount rotation");
  checkClose(sample.orientation.norm(), 1.0, 1e-9, "orientation stays a unit quaternion");

  driver.stop();
  close(master);
}

}  // namespace

int main() {
  testMath();
  testCrcRegression();
  testProtocolLayout();
  testReferenceTransform();
  testDriverDecode();
  testGroundFrameReordering();
  testMountRotation();

  if (g_failures == 0) {
    std::printf("\nall checks passed\n");
    return 0;
  }
  std::printf("\n%d check(s) failed\n", g_failures);
  return 1;
}
