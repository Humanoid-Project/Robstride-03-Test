# cpp_n100

## Build

```bash
cd src/cpp_n100
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Run

```bash
# or add yourself to the dialout group
sudo chmod 666 /dev/ttyUSB0
```

```bash
# Sensor state and link statistics, printed at 10 Hz
./build/read_imu /dev/ttyUSB0 921600

# Mount rotation as roll pitch yaw in degrees
./build/read_imu /dev/ttyUSB0 921600 180 0 0

# Print rate in Hz. Does not change the device stream, which is fixed near 100 Hz
./build/read_imu /dev/ttyUSB0 921600 --rate 2
./build/read_imu /dev/ttyUSB0 921600 180 0 0 --rate 20

# 50 Hz control loop, prints a timing summary once a second
./build/rl_observation /dev/ttyUSB0 921600 50
```

## Using it

```cpp
#include "n100/imu_driver.hpp"

n100::DriverConfig config;
config.port = "/dev/ttyUSB0";
config.baudrate = 921600;
// Rotation from the IMU case to the robot base. Stand the robot upright and run
// read_imu: gproj must read (0, 0, -1). If it reads (0, 0, +1), use Rx(pi).
config.mount_rotation = n100::Quat::fromAxisAngleX(M_PI);

n100::ImuDriver driver(config);
driver.start();                                            // throws on failure

n100::ImuSample warmup;
driver.waitForSample(warmup, std::chrono::seconds(3));
driver.calibrateGyroBias(std::chrono::milliseconds(2000)); // robot must be still
driver.resetStats();                                       // clear startup resync

// Control loop: latest() is a mutex protected copy, it never touches the port.
n100::ImuSample s;
if (driver.latest(s)) {
  const double obs[6] = {
      s.projected_gravity.x, s.projected_gravity.y, s.projected_gravity.z,
      s.angular_velocity.x,  s.angular_velocity.y,  s.angular_velocity.z};
  // feed obs to the policy
}
```

```cmake
add_subdirectory(path/to/cpp_n100)
target_link_libraries(my_controller PRIVATE n100::n100)
```

## Frame conventions

| Quantity | Frame | Unit |
| --- | --- | --- |
| `orientation` | base -> reference | unit quaternion |
| `angular_velocity` | base | rad/s |
| `linear_acceleration` | base | m/s^2 |
| `magnetic_field` | base | Tesla |
| `projected_gravity` | base | unit vector, `(0, 0, -1)` upright |

`base` is the IMU case frame when `mount_rotation` is identity, the robot base
frame otherwise.