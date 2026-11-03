#ifndef INCLUDE_IAHRS_ROS2_DRIVER_SERIAL
#define INCLUDE_IAHRS_ROS2_DRIVER_SERIAL

#include <string>

#define SERIAL_READ_LINE_NO_DATA 0
#define SERIAL_READ_LINE_ERROR -1
#define SERIAL_READ_LINE_TIMEOUT -2

class Serial {
public:
  Serial(const std::string &port, unsigned int baud_rate);
  ~Serial();

  bool open();
  void close();

  void flush();
  int read(char *buffer, size_t size);
  int read_until(char *buffer, const char *until, int timeout_ms);
  int read_line(char *buffer, int timeout_ms);

  int write(const char *data, size_t size);

private:
  std::string port_;
  unsigned int baud_rate_;
  int serial_fd_;
};

#endif /* INCLUDE_IAHRS_ROS2_DRIVER_SERIAL */
