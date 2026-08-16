// Thin RAII wrapper around a Linux tty, replacing the ROS `serial` package.
#ifndef N100_SERIAL_PORT_HPP_
#define N100_SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace n100 {

class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  // Opens the port as raw 8N1 with no flow control. Throws std::runtime_error
  // on failure. `low_latency` asks the driver to drop its receive coalescing
  // timer, which matters on FT232/CP210x bridges.
  void open(const std::string& device, int baudrate, bool low_latency = true);

  void close();
  bool isOpen() const { return fd_ >= 0; }

  // Waits up to `timeout_ms` for data, then reads whatever is available.
  // Returns the byte count, 0 on timeout. Throws std::runtime_error on a
  // hard I/O error such as the adapter being unplugged.
  std::size_t read(std::uint8_t* buffer, std::size_t size, int timeout_ms);

  // Discards buffered input and output.
  void flush();

  const std::string& device() const { return device_; }

 private:
  int fd_ = -1;
  std::string device_;
};

}  // namespace n100

#endif  // N100_SERIAL_PORT_HPP_
