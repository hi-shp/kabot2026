#ifndef INCLUDE_IAHRS_ROS2_DRIVER_IMU_DRIVER
#define INCLUDE_IAHRS_ROS2_DRIVER_IMU_DRIVER

#include <stdbool.h>
#include <stdint.h>

#include "serial.hpp"

#include "iahrs_obj.hpp"

typedef struct {
  double x;
  double y;
  double z;
} axis_data_t;

typedef struct {
  double x;
  double y;
  double z;
  double w;
} orientation_data_t;

typedef struct {
  uint64_t one_ms_time;                // IAHRS_DRIVER_SYNC_FLAG_1MS_TIME, 데이터시트를 통해 타입 알 수 없음.
  double temperature;                  // IAHRS_DRIVER_SYNC_FLAG_TEMP
  axis_data_t accel;                   // IAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL
  axis_data_t gyro;                    // IAHRS_DRIVER_SYNC_FLAG_SENSOR_GYRO
  axis_data_t mag;                     // IAHRS_DRIVER_SYNC_FLAG_SENSOR_MAG
  axis_data_t accel_gravity_removed;   // IAHRS_DRIVER_SYNC_FLAG_GRAVITY_REMOVED_ACCEL
  axis_data_t euler_angle;             // IAHRS_DRIVER_SYNC_FLAG_EULER_ANGLE
  orientation_data_t quaternion_angle; // IAHRS_DRIVER_SYNC_FLAG_QUATERNION
  axis_data_t velocity;                // IAHRS_DRIVER_SYNC_FLAG_GLOBAL_VELOCITY
  axis_data_t position;                // IAHRS_DRIVER_SYNC_FLAG_GLOBAL_POSITION
  axis_data_t vibration;               // IAHRS_DRIVER_SYNC_FLAG_VIBRATION
} iahrs_driver_sync_data_t;

typedef struct {
  struct {
    int major;
    int minor;
  } sw;

  struct {
    int major;
    int minor;
  } hw;
} iahrs_version_t;

class IAHRSDriver {
public:
  IAHRSDriver(const std::string &port);
  ~IAHRSDriver();

  bool initialize(void);
  bool reboot(void);

  bool set_sync(uint16_t target_mask, uint16_t period_ms);
  bool set_option(bool state);

  bool reset_euler_angle(void);

  bool fetch_sync_data(iahrs_driver_sync_data_t &data_buff);

  inline iahrs_version_t get_version(void) {
    return version_;
  }

  inline iahrs_driver_sync_flag_t get_sync_flag(void) {
    return (iahrs_driver_sync_flag_t)sync_mask_;
  }

private:
  Serial serial_;
  iahrs_version_t version_;
  uint16_t sync_mask_;

  char buff_[200];

  bool send_obj_(const std::string &obj);
  bool send_obj_(const std::string &obj, const std::string &data);

  uint64_t parse_sync_data_and_shift_buffer_uint64_(char *&buff);
  double parse_sync_data_and_shift_buffer_double_(char *&buff);

  /**
   * @brief buffer가 until과 동일한 문자열로 시작되는 경우 문자열이 종료될 때 까지 buffer 포인터 증가해 반환
   *
   * @param buffer
   * @param until
   * @return char*
   */
  inline char *move_ptr_until_end_(char *buffer, const char *until) {
    if (!buffer || !until) {
      return nullptr;
    }

    while (*until) {
      if (!*buffer || *buffer != *until) {
        return nullptr;
      }
      buffer++;
      until++;
    }
    return buffer;
  }

  inline void report_read_error(int ret) {
    if (ret == SERIAL_READ_LINE_NO_DATA) {
      std::cerr << "SERIAL_READ_LINE_NO_DATA" << std::endl;
    } else if (ret == SERIAL_READ_LINE_ERROR) {
      std::cerr << "SERIAL_READ_LINE_ERROR" << std::endl;
    } else if (ret == SERIAL_READ_LINE_TIMEOUT) {
      std::cerr << "SERIAL_READ_LINE_TIMEOUT" << std::endl;
    }
  }
};

#endif /* INCLUDE_IAHRS_ROS2_DRIVER_IMU_DRIVER */
