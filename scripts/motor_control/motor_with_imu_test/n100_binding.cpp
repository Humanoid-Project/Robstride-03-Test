// Python bindings for the N100 IMU SDK in n100_cpp/.
//
// The reader thread stays in C++ and never calls back into Python, so it runs
// free of the GIL. Every call that can block (stop, wait_for_sample,
// calibrate_gyro_bias) releases the GIL, which is what keeps a 100 Hz CAN
// control loop in the interpreter from stalling on the IMU.
//
// setSampleCallback is deliberately not exposed: invoking Python from the
// reader thread would force it to take the GIL on every frame and hand the
// control loop's jitter over to the interpreter. Poll latest() instead.

#include <pybind11/pybind11.h>

#include <chrono>
#include <sstream>
#include <string>

#include "n100/crc.hpp"
#include "n100/imu_driver.hpp"

namespace py = pybind11;

namespace {

std::chrono::milliseconds toMillis(double seconds) {
  if (seconds < 0.0) seconds = 0.0;
  return std::chrono::milliseconds(static_cast<std::int64_t>(seconds * 1000.0));
}

std::string vec3Repr(const n100::Vec3& v) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(6);
  out << "Vec3(x=" << v.x << ", y=" << v.y << ", z=" << v.z << ")";
  return out.str();
}

std::string quatRepr(const n100::Quat& q) {
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(6);
  out << "Quat(w=" << q.w << ", x=" << q.x << ", y=" << q.y << ", z=" << q.z << ")";
  return out.str();
}

}  // namespace

PYBIND11_MODULE(n100, m) {
  m.doc() = "WHEELTEC N100 IMU driver (bindings over n100_cpp/)";

  py::class_<n100::Vec3>(m, "Vec3")
      .def(py::init<>())
      .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("z"))
      .def_readwrite("x", &n100::Vec3::x)
      .def_readwrite("y", &n100::Vec3::y)
      .def_readwrite("z", &n100::Vec3::z)
      .def("norm", &n100::Vec3::norm)
      // Makes tuple(v), list(v) and numpy.array(v) all work.
      .def("__len__", [](const n100::Vec3&) { return 3; })
      .def("__getitem__",
           [](const n100::Vec3& v, std::size_t i) {
             if (i == 0) return v.x;
             if (i == 1) return v.y;
             if (i == 2) return v.z;
             throw py::index_error("Vec3 index out of range");
           })
      .def("__repr__", &vec3Repr);

  py::class_<n100::Euler>(m, "Euler")
      .def(py::init<>())
      .def_readwrite("roll", &n100::Euler::roll)
      .def_readwrite("pitch", &n100::Euler::pitch)
      .def_readwrite("yaw", &n100::Euler::yaw)
      .def("__repr__", [](const n100::Euler& e) {
        std::ostringstream out;
        out.setf(std::ios::fixed);
        out.precision(6);
        out << "Euler(roll=" << e.roll << ", pitch=" << e.pitch << ", yaw=" << e.yaw << ")";
        return out.str();
      });

  py::class_<n100::Quat>(m, "Quat")
      .def(py::init<>())
      .def(py::init<double, double, double, double>(), py::arg("w"), py::arg("x"),
           py::arg("y"), py::arg("z"))
      .def_readwrite("w", &n100::Quat::w)
      .def_readwrite("x", &n100::Quat::x)
      .def_readwrite("y", &n100::Quat::y)
      .def_readwrite("z", &n100::Quat::z)
      .def("norm", &n100::Quat::norm)
      .def("normalized", &n100::Quat::normalized)
      .def("conjugate", &n100::Quat::conjugate)
      .def("rotate", &n100::Quat::rotate, py::arg("v"),
           "Body frame vector -> reference frame.")
      .def("inverse_rotate", &n100::Quat::inverseRotate, py::arg("v"),
           "Reference frame vector -> body frame.")
      .def_static("from_axis_angle_x", &n100::Quat::fromAxisAngleX, py::arg("angle"))
      .def_static("from_axis_angle_y", &n100::Quat::fromAxisAngleY, py::arg("angle"))
      .def_static("from_axis_angle_z", &n100::Quat::fromAxisAngleZ, py::arg("angle"))
      .def_static("from_euler_zyx", &n100::Quat::fromEulerZYX, py::arg("roll"),
                  py::arg("pitch"), py::arg("yaw"),
                  "Intrinsic Z-Y-X composition, angles in radians.")
      .def("__repr__", &quatRepr);

  // Exposed so tests can synthesise valid frames without a device attached.
  m.def("crc8",
        [](const std::string& data) {
          return n100::crc8(reinterpret_cast<const std::uint8_t*>(data.data()), data.size());
        },
        py::arg("data"), "CRC8 over the 4 byte frame header.");
  m.def("crc16",
        [](const std::string& data) {
          return n100::crc16(reinterpret_cast<const std::uint8_t*>(data.data()), data.size());
        },
        py::arg("data"), "CRC-16/XMODEM over the frame payload.");

  m.def("to_euler", &n100::toEuler, py::arg("q"),
        "Intrinsic Z-Y-X decomposition, radians.");
  m.def("projected_gravity", &n100::projectedGravity, py::arg("orientation"),
        "Gravity direction in the body frame; (0, 0, -1) when upright.");

  py::class_<n100::ImuSample>(m, "ImuSample")
      .def(py::init<>())
      .def_readonly("orientation", &n100::ImuSample::orientation)
      .def_readonly("euler", &n100::ImuSample::euler)
      .def_readonly("angular_velocity", &n100::ImuSample::angular_velocity)
      .def_readonly("angular_velocity_raw", &n100::ImuSample::angular_velocity_raw)
      .def_readonly("linear_acceleration", &n100::ImuSample::linear_acceleration)
      .def_readonly("magnetic_field", &n100::ImuSample::magnetic_field)
      .def_readonly("projected_gravity", &n100::ImuSample::projected_gravity)
      .def_readonly("imu_temperature", &n100::ImuSample::imu_temperature)
      .def_readonly("pressure", &n100::ImuSample::pressure)
      .def_readonly("device_timestamp_us", &n100::ImuSample::device_timestamp_us)
      .def_readonly("host_timestamp_ns", &n100::ImuSample::host_timestamp_ns)
      .def_readonly("seq", &n100::ImuSample::seq)
      .def_readonly("has_imu_frame", &n100::ImuSample::has_imu_frame)
      .def("__repr__", [](const n100::ImuSample& s) {
        std::ostringstream out;
        out.setf(std::ios::fixed);
        out.precision(4);
        out << "ImuSample(seq=" << s.seq << ", gproj=(" << s.projected_gravity.x << ", "
            << s.projected_gravity.y << ", " << s.projected_gravity.z << "))";
        return out.str();
      });

  py::class_<n100::DriverStats>(m, "DriverStats")
      .def(py::init<>())
      .def_readonly("imu_frames", &n100::DriverStats::imu_frames)
      .def_readonly("ahrs_frames", &n100::DriverStats::ahrs_frames)
      .def_readonly("insgps_frames", &n100::DriverStats::insgps_frames)
      .def_readonly("ground_frames", &n100::DriverStats::ground_frames)
      .def_readonly("samples", &n100::DriverStats::samples)
      .def_readonly("crc8_errors", &n100::DriverStats::crc8_errors)
      .def_readonly("crc16_errors", &n100::DriverStats::crc16_errors)
      .def_readonly("frame_end_errors", &n100::DriverStats::frame_end_errors)
      .def_readonly("dropped_bytes", &n100::DriverStats::dropped_bytes)
      .def_readonly("sn_lost", &n100::DriverStats::sn_lost)
      .def_readonly("bytes_read", &n100::DriverStats::bytes_read)
      .def("__repr__", [](const n100::DriverStats& s) {
        std::ostringstream out;
        out << "DriverStats(samples=" << s.samples << ", crc8=" << s.crc8_errors
            << ", crc16=" << s.crc16_errors << ", sn_lost=" << s.sn_lost
            << ", dropped=" << s.dropped_bytes << ")";
        return out.str();
      });

  py::class_<n100::DriverConfig>(m, "DriverConfig")
      .def(py::init([](std::string port, int baudrate, bool low_latency,
                       bool apply_reference_frame_transform, n100::Quat mount_rotation,
                       n100::Vec3 magnetometer_offset, n100::Vec3 gyro_bias,
                       int read_timeout_ms) {
             n100::DriverConfig c;
             c.port = std::move(port);
             c.baudrate = baudrate;
             c.low_latency = low_latency;
             c.apply_reference_frame_transform = apply_reference_frame_transform;
             c.mount_rotation = mount_rotation;
             c.magnetometer_offset = magnetometer_offset;
             c.gyro_bias = gyro_bias;
             c.read_timeout_ms = read_timeout_ms;
             return c;
           }),
           py::arg("port") = "/dev/ttyUSB0", py::arg("baudrate") = 921600,
           py::arg("low_latency") = true,
           py::arg("apply_reference_frame_transform") = true,
           py::arg("mount_rotation") = n100::Quat(),
           py::arg("magnetometer_offset") = n100::Vec3(),
           py::arg("gyro_bias") = n100::Vec3(), py::arg("read_timeout_ms") = 20)
      .def_readwrite("port", &n100::DriverConfig::port)
      .def_readwrite("baudrate", &n100::DriverConfig::baudrate)
      .def_readwrite("low_latency", &n100::DriverConfig::low_latency)
      .def_readwrite("apply_reference_frame_transform",
                     &n100::DriverConfig::apply_reference_frame_transform)
      .def_readwrite("mount_rotation", &n100::DriverConfig::mount_rotation)
      .def_readwrite("magnetometer_offset", &n100::DriverConfig::magnetometer_offset)
      .def_readwrite("gyro_bias", &n100::DriverConfig::gyro_bias)
      .def_readwrite("read_timeout_ms", &n100::DriverConfig::read_timeout_ms);

  py::class_<n100::ImuDriver>(m, "ImuDriver")
      .def(py::init<n100::DriverConfig>(), py::arg("config") = n100::DriverConfig())
      .def("start", &n100::ImuDriver::start,
           "Open the port and start the reader thread. Raises RuntimeError on failure.")
      .def("stop", &n100::ImuDriver::stop, py::call_guard<py::gil_scoped_release>(),
           "Stop the reader thread and close the port. Safe to call twice.")
      .def_property_readonly("is_running", &n100::ImuDriver::isRunning)
      .def("latest",
           [](const n100::ImuDriver& d) -> py::object {
             n100::ImuSample sample;
             bool ok;
             {
               py::gil_scoped_release release;
               ok = d.latest(sample);
             }
             return ok ? py::cast(sample) : py::none();
           },
           "Most recent sample, or None before the first one arrives. Never "
           "touches the serial port, so it is safe inside a control loop.")
      .def("wait_for_sample",
           [](const n100::ImuDriver& d, double timeout, std::uint64_t last_seq) -> py::object {
             n100::ImuSample sample;
             bool ok;
             {
               py::gil_scoped_release release;
               ok = d.waitForSample(sample, toMillis(timeout), last_seq);
             }
             return ok ? py::cast(sample) : py::none();
           },
           py::arg("timeout"), py::arg("last_seq") = 0,
           "Block until a sample newer than last_seq arrives. `timeout` is in "
           "seconds. Returns None on timeout or if the driver stopped.")
      .def("stats", &n100::ImuDriver::stats)
      .def("reset_stats", &n100::ImuDriver::resetStats,
           "Zero the counters, e.g. after the one time startup resynchronisation.")
      .def("calibrate_gyro_bias",
           [](n100::ImuDriver& d, double duration) {
             py::gil_scoped_release release;
             return d.calibrateGyroBias(toMillis(duration));
           },
           py::arg("duration") = 2.0,
           "Average the gyro over `duration` seconds and install it as the bias. "
           "The device must be completely still. Measures both the fused and the "
           "raw bias in one pass.")
      .def_property("gyro_bias", &n100::ImuDriver::gyroBias, &n100::ImuDriver::setGyroBias,
                    "Bias of the AHRS fused rates, i.e. of sample.angular_velocity.")
      .def_property("gyro_bias_raw", &n100::ImuDriver::gyroBiasRaw,
                    &n100::ImuDriver::setGyroBiasRaw,
                    "Bias of the raw IMU frame gyro, i.e. of sample.angular_velocity_raw.")
      .def("last_error", &n100::ImuDriver::lastError,
           "Reason the reader thread aborted, empty string otherwise.")
      .def_property_readonly("config", &n100::ImuDriver::config)
      // `with n100.ImuDriver(cfg) as driver:` guarantees the port is released
      // even when the control loop raises.
      .def("__enter__",
           [](n100::ImuDriver& d) {
             d.start();
             return &d;
           },
           py::return_value_policy::reference_internal)
      .def("__exit__", [](n100::ImuDriver& d, py::object, py::object, py::object) {
        py::gil_scoped_release release;
        d.stop();
        return false;
      });
}
