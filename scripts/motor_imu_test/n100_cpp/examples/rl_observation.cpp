// Fixed rate control loop that builds the IMU part of a walking policy
// observation, the way a deployed policy would consume this SDK.
//
//   ./rl_observation [port] [baudrate] [rate_hz]
//
// Observation layout (6 values, the usual Isaac Lab convention):
//   [0..2] projected gravity, body frame, unit vector
//   [3..5] angular velocity, body frame, rad/s

#include <algorithm>
#include <array>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

#include "n100/imu_driver.hpp"

namespace {
volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }
}  // namespace

int main(int argc, char** argv) {
  n100::DriverConfig config;
  if (argc > 1) config.port = argv[1];
  if (argc > 2) config.baudrate = std::atoi(argv[2]);
  const double rate_hz = argc > 3 ? std::atof(argv[3]) : 50.0;
  if (rate_hz <= 0.0) {
    std::fprintf(stderr, "rate must be positive\n");
    return 1;
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

  // Wait for the stream to come up before touching the gyro bias.
  n100::ImuSample warmup;
  if (!driver.waitForSample(warmup, std::chrono::seconds(3))) {
    std::fprintf(stderr, "no IMU data within 3 s\n");
    return 1;
  }

  std::printf("calibrating gyro bias, keep the IMU still...\n");
  if (driver.calibrateGyroBias(std::chrono::milliseconds(2000))) {
    const n100::Vec3 b = driver.gyroBias();
    std::printf("gyro bias [rad/s]: % .5f % .5f % .5f\n\n", b.x, b.y, b.z);
  } else {
    std::fprintf(stderr, "gyro calibration failed, continuing with zero bias\n\n");
  }

  // Opening the port mid-stream costs a one time resynchronisation. Clear it so
  // the counters below only reflect faults during the control loop itself.
  driver.resetStats();

  const auto period = std::chrono::nanoseconds(
      static_cast<std::int64_t>(1e9 / rate_hz));
  auto next_tick = std::chrono::steady_clock::now();

  // Ticks between summary lines, about one second. Clamped to at least 1 so a
  // sub-1 Hz rate cannot turn the modulo below into a division by zero.
  const std::uint64_t report_interval =
      std::max<std::uint64_t>(1, static_cast<std::uint64_t>(rate_hz));

  std::vector<double> jitter_us;
  jitter_us.reserve(static_cast<std::size_t>(report_interval) * 2 + 8);
  std::uint64_t stale_ticks = 0;
  std::uint64_t last_seq = 0;
  std::uint64_t tick = 0;

  while (!g_stop && driver.isRunning()) {
    next_tick += period;
    std::this_thread::sleep_until(next_tick);

    const auto entered = std::chrono::steady_clock::now();
    jitter_us.push_back(
        std::chrono::duration<double, std::micro>(entered - next_tick).count());

    n100::ImuSample s;
    if (!driver.latest(s)) continue;
    if (s.seq == last_seq) ++stale_ticks;  // policy ran faster than the sensor
    last_seq = s.seq;

    std::array<double, 6> observation{
        s.projected_gravity.x, s.projected_gravity.y, s.projected_gravity.z,
        s.angular_velocity.x, s.angular_velocity.y, s.angular_velocity.z};

    // Age of the reading at the moment the policy would consume it.
    const double age_ms =
        (std::chrono::duration_cast<std::chrono::nanoseconds>(
             entered.time_since_epoch()).count() - s.host_timestamp_ns) / 1e6;

    // --- policy inference would go here, fed with `observation` ---

    if (++tick % report_interval == 0) {
      std::sort(jitter_us.begin(), jitter_us.end());
      const double p50 = jitter_us[jitter_us.size() / 2];
      const double p99 = jitter_us[(jitter_us.size() * 99) / 100];
      std::printf(
          "obs [% .3f % .3f % .3f | % .3f % .3f % .3f]  "
          "age %5.2f ms  jitter p50 %6.1f us p99 %7.1f us  stale %llu/%llu\n",
          observation[0], observation[1], observation[2],
          observation[3], observation[4], observation[5],
          age_ms, p50, p99,
          static_cast<unsigned long long>(stale_ticks),
          static_cast<unsigned long long>(tick));
      jitter_us.clear();
    }
  }

  if (!driver.lastError().empty()) {
    std::fprintf(stderr, "reader stopped: %s\n", driver.lastError().c_str());
  }
  driver.stop();
  return 0;
}
