// Prints the decoded IMU state and link statistics.
//
//   ./read_imu [port] [baudrate] [mount_roll_deg mount_pitch_deg mount_yaw_deg]
//              [--rate HZ]
//
// Use the mount angles to find the rotation from the IMU case to the robot
// base: with the robot upright, the correct values make gproj read (0, 0, -1).
//
// --rate sets how often the block below is printed, 10 Hz by default. It does
// not change how fast the device streams, which is fixed at about 100 Hz.

#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "n100/imu_driver.hpp"

namespace {
volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }
}  // namespace

int main(int argc, char** argv) {
  n100::DriverConfig config;
  double rate_hz = 10.0;

  // --rate may appear anywhere, so the positional arguments keep their meaning.
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--rate") {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "--rate needs a value in Hz\n");
        return 1;
      }
      rate_hz = std::atof(argv[++i]);
      continue;
    }
    positional.push_back(arg);
  }

  if (rate_hz <= 0.0) {
    std::fprintf(stderr, "--rate must be positive\n");
    return 1;
  }
  if (!positional.empty()) config.port = positional[0];
  if (positional.size() > 1) config.baudrate = std::atoi(positional[1].c_str());
  if (positional.size() == 3 || positional.size() == 4) {
    std::fprintf(stderr,
                 "the mount rotation needs all three angles: "
                 "roll pitch yaw, in degrees\n");
    return 1;
  }
  if (positional.size() > 4) {
    const double deg = 3.141592653589793 / 180.0;
    config.mount_rotation = n100::Quat::fromEulerZYX(std::atof(positional[2].c_str()) * deg,
                                                     std::atof(positional[3].c_str()) * deg,
                                                     std::atof(positional[4].c_str()) * deg);
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  n100::ImuDriver driver(config);
  try {
    driver.start();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "failed to start driver: %s\n", e.what());
    return 1;
  }
  std::printf("reading %s at %d baud, printing at %g Hz, Ctrl-C to stop\n\n",
              config.port.c_str(), config.baudrate, rate_hz);

  auto last_report = std::chrono::steady_clock::now();
  std::uint64_t last_samples = 0;

  // Absolute schedule, so printing time does not push the period out.
  const auto period = std::chrono::nanoseconds(
      static_cast<std::int64_t>(1e9 / rate_hz));
  auto next_tick = std::chrono::steady_clock::now();

  while (!g_stop && driver.isRunning()) {
    next_tick += period;
    std::this_thread::sleep_until(next_tick);

    n100::ImuSample s;
    if (!driver.latest(s)) {
      std::printf("waiting for data...\n");
      continue;
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - last_report).count();
    const n100::DriverStats st = driver.stats();
    const double rate = elapsed > 0.0 ? (st.samples - last_samples) / elapsed : 0.0;
    last_report = now;
    last_samples = st.samples;

    std::printf(
        "seq %8llu  %6.1f Hz\n"
        "  quat   w % .4f  x % .4f  y % .4f  z % .4f\n"
        "  rpy    r % 8.2f  p % 8.2f  y % 8.2f  [deg]\n"
        "  gyro   x % 8.4f  y % 8.4f  z % 8.4f  [rad/s, AHRS fused]\n"
        "  gyroR  x % 8.4f  y % 8.4f  z % 8.4f  [rad/s, raw]\n"
        "  accel  x % 8.4f  y % 8.4f  z % 8.4f  [m/s^2]\n"
        "  gproj  x % 8.4f  y % 8.4f  z % 8.4f\n"
        "  temp %5.1f C   crc8 %llu  crc16 %llu  sn_lost %llu  dropped %llu\n\n",
        static_cast<unsigned long long>(s.seq), rate,
        s.orientation.w, s.orientation.x, s.orientation.y, s.orientation.z,
        s.euler.roll * 180.0 / 3.141592653589793,
        s.euler.pitch * 180.0 / 3.141592653589793,
        s.euler.yaw * 180.0 / 3.141592653589793,
        s.angular_velocity.x, s.angular_velocity.y, s.angular_velocity.z,
        s.angular_velocity_raw.x, s.angular_velocity_raw.y, s.angular_velocity_raw.z,
        s.linear_acceleration.x, s.linear_acceleration.y, s.linear_acceleration.z,
        s.projected_gravity.x, s.projected_gravity.y, s.projected_gravity.z,
        s.imu_temperature,
        static_cast<unsigned long long>(st.crc8_errors),
        static_cast<unsigned long long>(st.crc16_errors),
        static_cast<unsigned long long>(st.sn_lost),
        static_cast<unsigned long long>(st.dropped_bytes));
  }

  if (!driver.lastError().empty()) {
    std::fprintf(stderr, "reader stopped: %s\n", driver.lastError().c_str());
  }
  driver.stop();
  return 0;
}
