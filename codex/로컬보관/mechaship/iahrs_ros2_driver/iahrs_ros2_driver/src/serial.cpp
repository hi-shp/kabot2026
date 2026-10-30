#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "iahrs_ros2_driver/serial.hpp"

Serial::Serial(const std::string &port, unsigned int baud_rate)
    : port_(port), baud_rate_(baud_rate), serial_fd_(-1) {}

Serial::~Serial() {
  close();
}

bool Serial::open() {
  // std::cerr << "Try to open serial: " << port_ << std::endl;

  serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_fd_ < 0) {
    // std::cerr << "Error opening serial port: " << port_ << std::endl;
    return false;
  }

  struct termios tio;
  if (tcgetattr(serial_fd_, &tio) < 0) {
    // std::cerr << "Error tcgetattr() function return error" << std::endl;
    close();
    return false;
  }

  cfmakeraw(&tio);
  tio.c_cflag = CS8 | CLOCAL | CREAD;
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_oflag &= ~OPOST;
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  cfsetspeed(&tio, baud_rate_);
  tio.c_cc[VTIME] = 1;
  tio.c_cc[VMIN] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tio) != 0) {
    // std::cerr << "Error tcsetattr() function return error" << std::endl;
    close();
    return false;
  }

  // std::cerr << "Serial port opened successfully" << std::endl;
  return true;
}

void Serial::close() {
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    // std::cerr << "Serial port closed" << std::endl;
  }
}

void Serial::flush() {
  if (serial_fd_ >= 0) {
    tcflush(serial_fd_, TCIOFLUSH); // 입력 및 출력 버퍼 비우기
  }
}

int Serial::read(char *buffer, size_t size) {
  if (serial_fd_ < 0 || buffer == nullptr) {
    return -1;
  }
  return ::read(serial_fd_, buffer, size);
}

int Serial::write(const char *data, size_t size) {
  if (serial_fd_ < 0 || data == nullptr) {
    return -1;
  }
  return ::write(serial_fd_, data, size);
}

int Serial::read_until(char *buffer, const char *until, int timeout_ms) {
  if (serial_fd_ < 0 || buffer == nullptr || until == nullptr) {
    return SERIAL_READ_LINE_ERROR;
  }

  int total_read = 0;
  char *current = buffer;
  int remaining_time = timeout_ms * 100;
  const int sleep_interval_us = 10; // polling interval
  int until_length = strlen(until);
  int match_index = 0;

  while (remaining_time >= 0) {
    int bytes_read = ::read(serial_fd_, current, 1); // read one byte at a time

    if (bytes_read > 0) {
      if (*current == until[match_index]) {
        match_index++;
        if (match_index == until_length) {
          // 'until' string found
          *(current + 1) = '\0'; // null-terminate the string
          return total_read + 1;
        }
      } else {
        match_index = 0; // reset match index if not matching
      }
      current++;
      total_read += bytes_read;
    }
    // else if (bytes_read < 0) {
    // read error
    // return SERIAL_READ_LINE_ERROR;
    // }

    // sleep for a short period to prevent busy waiting
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_interval_us));
    remaining_time -= sleep_interval_us;
  }

  // timeout occurred
  return SERIAL_READ_LINE_TIMEOUT;
}

int Serial::read_line(char *buffer, int timeout_ms) {
  if (serial_fd_ < 0 || buffer == nullptr) {
    return SERIAL_READ_LINE_ERROR;
  }

  int total_read = 0;
  char *current = buffer;
  int remaining_time = timeout_ms;
  const int sleep_interval_ms = 10; // polling interval

  while (remaining_time > 0) {
    int bytes_read = ::read(serial_fd_, current, 1); // read one byte at a time

    if (bytes_read > 0) {
      if (*current == '\n') {
        // newline character found
        *(current + 1) = '\0'; // null-terminate the string
        return total_read + 1;
      }
      current++;
      total_read += bytes_read;
    }
    // else if (bytes_read < 0) {
    // read error
    // return SERIAL_READ_LINE_ERROR;
    // }

    // sleep for a short period to prevent busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_interval_ms));
    remaining_time -= sleep_interval_ms;
  }

  // timeout occurred
  return SERIAL_READ_LINE_TIMEOUT;
}
