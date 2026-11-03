#include <chrono>
#include <iostream>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include <unistd.h>

#include "iahrs_ros2_driver/iahrs_driver.hpp"

#define END_DATA "\r\n\x00"

IAHRSDriver::IAHRSDriver(const std::string &port) : serial_(port, 115200) {}

IAHRSDriver::~IAHRSDriver() {
  reboot();
}

bool IAHRSDriver::send_obj_(const std::string &obj) {
  size_t ret = serial_.write(obj.c_str(), strlen(obj.c_str()) * sizeof(char));
  ret += serial_.write("\n", 1);

  if (ret != strlen(obj.c_str()) + 1) {
    std::cerr << "Failed to send." << std::endl;
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  serial_.flush(); // 반환값 무시

  return true;
}

bool IAHRSDriver::send_obj_(const std::string &obj, const std::string &data) {
  size_t ret = serial_.write(obj.c_str(), strlen(obj.c_str()) * sizeof(char));
  ret += serial_.write("=", 1);
  ret += serial_.write(data.c_str(), strlen(data.c_str()) * sizeof(char));
  ret += serial_.write("\n", 1);

  if (ret != strlen(obj.c_str()) + strlen(data.c_str()) + 2) {
    std::cerr << "Failed to send." << std::endl;
    return false;
  }

  ret = serial_.read_until(buff_, END_DATA, 500);
  if (ret <= 0) {
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  serial_.flush(); // 반환값 무시

  return true;
}

uint64_t IAHRSDriver::parse_sync_data_and_shift_buffer_uint64_(char *&buff) {
  uint64_t value;
  int offset = 0;

  sscanf(buff, "%lu,%n", &value, &offset);
  buff += offset;

  return value;
}

double IAHRSDriver::parse_sync_data_and_shift_buffer_double_(char *&buff) {
  double value;
  int offset = 0;

  sscanf(buff, "%lf,%n", &value, &offset);
  buff += offset;

  return value;
}

bool IAHRSDriver::initialize() {
  if (!serial_.open()) {
    return false;
  }

  return true;
}

bool IAHRSDriver::reboot() {
  serial_.flush();
  int ret = send_obj_(IAHRS_OBJ_SET_SENSOR_RESTART);
  if (ret == false) {
    return false;
  }

  char *temp_ptr;
  for (int i = 0; i < 4; i++) {
    ret = serial_.read_until(buff_, END_DATA, 500);
    if (ret < 1) {
      report_read_error(ret);
      return false;
    }

    switch (i) {
    case 1:
      if (strcmp(buff_, "iAHRS" END_DATA) != 0) {
        return false;
      }
      break;

    case 2:
      temp_ptr = move_ptr_until_end_(buff_, "S/W ver: ");
      if (temp_ptr == nullptr) {
        return false;
      }
      sscanf(temp_ptr, "%d.%d", &version_.sw.major, &version_.sw.minor);
      break;

    case 3:
      temp_ptr = move_ptr_until_end_(buff_, "H/W ver: ");
      if (temp_ptr == nullptr) {
        return false;
      }
      sscanf(temp_ptr, "%d.%d", &version_.hw.major, &version_.hw.minor);
      break;

    case 0:
    default:
      break;
    }
  }
  return true;
}

bool IAHRSDriver::set_sync(uint16_t target_mask, uint16_t period_ms) {
  if (target_mask >= IAHRS_DRIVER_SYNC_FLAG_MAX) {
    return false;
  }
  sync_mask_ = target_mask;
  bool ret = true;

  ret &= send_obj_(IAHRS_OBJ_SETW_SYNC_DATA_MODE, "1");
  sprintf(buff_, "%d", period_ms);
  ret &= send_obj_(IAHRS_OBJ_SETW_SYNC_DATA_PERIOD, buff_);
  sprintf(buff_, "0x%x", target_mask);
  ret &= send_obj_(IAHRS_OBJ_SETW_SYNC_DATA_TYPE, buff_);

  return ret;
}

bool IAHRSDriver::set_option(bool state) {
  return send_obj_(IAHRS_OBJ_SETW_OPTION, state ? "1" : "0");
}

bool IAHRSDriver::reset_euler_angle(void) {
  return send_obj_(IAHRS_OBJ_SET_RESET_EULER_ANGLE);
}

bool IAHRSDriver::fetch_sync_data(iahrs_driver_sync_data_t &data_buff) {
  int ret;

  ret = serial_.read_until(buff_, END_DATA, 100);
  if (ret <= 0) {
    return false;
  }
  char *buff_temp_ptr = buff_;
  for (iahrs_driver_sync_flag_t i = static_cast<iahrs_driver_sync_flag_t>(IAHRS_DRIVER_SYNC_FLAG_NONE + 1);
       i < IAHRS_DRIVER_SYNC_FLAG_MAX;
       i = static_cast<iahrs_driver_sync_flag_t>(i << 1)) {

    if ((sync_mask_ & i) != i) {
      continue;
    }

    switch (i) {
    case IAHRS_DRIVER_SYNC_FLAG_1MS_TIME:
      data_buff.one_ms_time = parse_sync_data_and_shift_buffer_uint64_(buff_temp_ptr);
      break;

    case IAHRS_DRIVER_SYNC_FLAG_TEMP:
      data_buff.temperature = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      break;

    case IAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL:
      data_buff.accel.x = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.accel.y = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.accel.z = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      break;

    case IAHRS_DRIVER_SYNC_FLAG_SENSOR_GYRO:
      data_buff.gyro.x = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.gyro.y = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.gyro.z = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      break;

    case IAHRS_DRIVER_SYNC_FLAG_SENSOR_MAG:
      data_buff.mag.x = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.mag.y = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.mag.z = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      break;

    case IAHRS_DRIVER_SYNC_FLAG_GRAVITY_REMOVED_ACCEL:
      data_buff.accel_gravity_removed.x = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.accel_gravity_removed.y = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.accel_gravity_removed.z = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      break;

    case IAHRS_DRIVER_SYNC_FLAG_EULER_ANGLE:
      data_buff.euler_angle.x = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.euler_angle.y = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.euler_angle.z = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      break;

    case IAHRS_DRIVER_SYNC_FLAG_QUATERNION:
      data_buff.quaternion_angle.w = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr); // r
      data_buff.quaternion_angle.x = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr); // v0
      data_buff.quaternion_angle.y = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr); // v1
      data_buff.quaternion_angle.z = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr); // v2
      break;

    case IAHRS_DRIVER_SYNC_FLAG_GLOBAL_VELOCITY:
      data_buff.velocity.x = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.velocity.y = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.velocity.z = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      break;

    case IAHRS_DRIVER_SYNC_FLAG_GLOBAL_POSITION:
      data_buff.position.x = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.position.y = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.position.z = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      break;

    case IAHRS_DRIVER_SYNC_FLAG_VIBRATION:
      data_buff.vibration.x = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.vibration.y = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      data_buff.vibration.z = parse_sync_data_and_shift_buffer_double_(buff_temp_ptr);
      break;

    default:
      return false;
    }
  }

  return true;
}
