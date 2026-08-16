#include "n100/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#ifdef __linux__
#include <linux/serial.h>
#include <sys/ioctl.h>
#endif

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace n100 {
namespace {

speed_t toSpeed(int baudrate) {
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 3000000: return B3000000;
    case 4000000: return B4000000;
    default: return 0;
  }
}

std::string errnoMessage(const std::string& what) {
  return what + ": " + std::strerror(errno);
}

// Best effort: tells FTDI/CP210x style drivers not to hold received bytes
// waiting for a full USB frame. Silently ignored when unsupported.
void requestLowLatency(int fd) {
#ifdef __linux__
  struct serial_struct info;
  if (ioctl(fd, TIOCGSERIAL, &info) != 0) return;
  info.flags |= ASYNC_LOW_LATENCY;
  ioctl(fd, TIOCSSERIAL, &info);
#else
  (void)fd;
#endif
}

}  // namespace

SerialPort::~SerialPort() { close(); }

void SerialPort::open(const std::string& device, int baudrate, bool low_latency) {
  close();

  const speed_t speed = toSpeed(baudrate);
  if (speed == 0) {
    throw std::runtime_error("unsupported baudrate: " + std::to_string(baudrate));
  }

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error(errnoMessage("failed to open " + device));
  }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0) {
    const std::string message = errnoMessage("tcgetattr failed on " + device);
    close();
    throw std::runtime_error(message);
  }

  cfmakeraw(&tty);
  tty.c_cflag |= CLOCAL | CREAD;   // ignore modem lines, enable the receiver
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;              // 8 data bits
  tty.c_cflag &= ~PARENB;          // no parity
  tty.c_cflag &= ~CSTOPB;          // 1 stop bit
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;         // no hardware flow control
#endif
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // no software flow control

  // Blocking behaviour is handled by poll(), so make read() return immediately.
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
    const std::string message = errnoMessage("failed to set baudrate on " + device);
    close();
    throw std::runtime_error(message);
  }

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const std::string message = errnoMessage("tcsetattr failed on " + device);
    close();
    throw std::runtime_error(message);
  }

  if (low_latency) requestLowLatency(fd_);

  device_ = device;
  flush();
}

void SerialPort::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  device_.clear();
}

void SerialPort::flush() {
  if (fd_ >= 0) tcflush(fd_, TCIOFLUSH);
}

std::size_t SerialPort::read(std::uint8_t* buffer, std::size_t size, int timeout_ms) {
  if (fd_ < 0) throw std::runtime_error("serial port is not open");
  if (size == 0) return 0;

  struct pollfd pfd;
  pfd.fd = fd_;
  pfd.events = POLLIN;
  pfd.revents = 0;

  const int ready = ::poll(&pfd, 1, timeout_ms);
  if (ready < 0) {
    if (errno == EINTR) return 0;
    throw std::runtime_error(errnoMessage("poll failed on " + device_));
  }
  if (ready == 0) return 0;

  if (pfd.revents & (POLLERR | POLLNVAL)) {
    throw std::runtime_error("serial port error on " + device_);
  }
  if ((pfd.revents & POLLHUP) && !(pfd.revents & POLLIN)) {
    throw std::runtime_error("serial port disconnected: " + device_);
  }

  const ssize_t count = ::read(fd_, buffer, size);
  if (count < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
    throw std::runtime_error(errnoMessage("read failed on " + device_));
  }
  return static_cast<std::size_t>(count);
}

}  // namespace n100
