#ifndef INCLUDE_IAHRS_ROS2_DRIVER_IAHRS_OBJ
#define INCLUDE_IAHRS_ROS2_DRIVER_IAHRS_OBJ

#define IAHRS_OBJ_GET_ID "vr"                                    // 제품 ID와 버전 정보 표시
#define IAHRS_OBJ_GET_INFO "h"                                   // 도움말 표시
#define IAHRS_OBJ_GET_INFO_FILTER "hh"                           // 도움말 표시 (필터링된 정보)
#define IAHRS_OBJ_GET_INPUT_VOLTAGE "pv"                         // 입력 전압 표시
#define IAHRS_OBJ_GET_SENSOR_TEMP "t"                            // 센서 온도 표시
#define IAHRS_OBJ_GET_ACCELERATION "a"                           // 센서 좌표계 기준 가속도 표시 (ax, ay, az)
#define IAHRS_OBJ_GET_ANGULAR_VELOCITY "g"                       // 센서 좌표계 기준 각속도 표시 (ωx, ωy, ωz)
#define IAHRS_OBJ_GET_MAGNETIC_FIELD "m"                         // 센서 좌표계 기준 지자기 표시 (mx, my, mz)
#define IAHRS_OBJ_GET_EULER_ANGLE "e"                            // Euler angle 표시 (roll, pitch, yaw), Z-Y-X 회전
#define IAHRS_OBJ_GET_QUATERNION "q"                             // Quaternion 표시 (r, v0, v1, v2)
#define IAHRS_OBJ_GET_LINEAR_ACCELERATION "ag"                   // 전역 좌표계 기준 중력을 제거한 가속도 표시 (ax, ay, az)
#define IAHRS_OBJ_GET_VELOCITY "v"                               // 전역 좌표계 기준 속도 표시 (vx, vy, vz)
#define IAHRS_OBJ_GET_POSITION "p"                               // 전역 좌표계 기준 위치 표시 (px, py, pz)
#define IAHRS_OBJ_GET_VIBRATION "f"                              // 가속도 또는 각속도의 진동과 크기 표시 (fx, mx, fy, my, fz, mz)
#define IAHRS_OBJ_GET_VIBRATION_DETAILS "vd"                     // 진동의 주파수와 댐핑 계수, 크기 표시
#define IAHRS_OBJ_GET_STATUS "s"                                 // 센서 상태 표시
#define IAHRS_OBJ_SETW_CONTROL "c"                               // 센서 제어 명령
#define IAHRS_OBJ_SETW_OPTION "o"                                // 센서 옵션 설정 -> 16진수 반환 (0x0 or 0x1)
#define IAHRS_OBJ_SETW_RS232_BAUDRATE "b1"                       // RS-232 포트의 baudrate 설정
#define IAHRS_OBJ_SETW_USB_UART_BAUDRATE "b2"                    // USB/UART 포트의 baudrate 설정
#define IAHRS_OBJ_SETW_SYNC_DATA_MODE "so"                       // Sync. data 전송 포트 설정 (0 = RS-232, 1 = USB/UART) -> 2진수 반환 (0b0 or 0b1)
#define IAHRS_OBJ_SETW_SYNC_DATA_PERIOD "sp"                     // Sync. data 전송 주기 설정 (1 ~ 60000 ms) -> 10진수 반환
#define IAHRS_OBJ_SETW_SYNC_DATA_TYPE "sd"                       // Sync. data 종류 선택 (0x0000 ~ 0xFFFF) -> 16진수 반환 (가변 길이)
#define IAHRS_OBJ_SETW_FFT_SOURCE "fs"                           // FFT 소스 선택 (0 = none, 1 ~ 4 = Accl. or Gyro. input)
#define IAHRS_OBJ_SETW_VIBRATION_SOURCE "vs"                     // Vibration 소스 선택 (0 = none, 1 ~ 2 = Accl. or Gyro. input)
#define IAHRS_OBJ_SETW_FFT_INPUT_SCALE "at"                      // FFT/Vibration 입력 가속도 크기 임계값 (0 ~ 160) 설정
#define IAHRS_OBJ_SETW_FFT_INPUT_SCALE_DEG "gt"                  // FFT/Vibration 입력 각속도 크기 임계값 (0 ~ 2000) 설정
#define IAHRS_OBJ_SET_FLASH_MEMORY_SAVE "fw"                     // c=1, 플래시 메모리에 설정값 저장
#define IAHRS_OBJ_SET_FACTORY_RESET "fd"                         // c=2, 제품 출하시 초기 설정값 복구
#define IAHRS_OBJ_SET_ACCEL_GYRO_BIAS_SCALE "ca"                 // c=3, 가속도와 각속도 센서의 바이어스/스케일 보정
#define IAHRS_OBJ_SET_MAGNETIC_BIAS_SCALE "cm"                   // c=4, 지자기 센서의 바이어스와 스케일 보정
#define IAHRS_OBJ_SET_EULER_ANGLE_ZERO "za"                      // c=5, 현재 Euler angle을 0위치로 설정
#define IAHRS_OBJ_SET_RESET_EULER_ANGLE "ra"                     // c=7, Euler angle 초기화
#define IAHRS_OBJ_SET_RESET_POSITION_SPEED "rp"                  // c=8, 위치와 속도 초기화
#define IAHRS_OBJ_SET_RESET_ALL_DATA "rc"                        // c=9, 모든 보정 데이터 초기화
#define IAHRS_OBJ_SET_SENSOR_RESTART "rd"                        // c=99, 센서 재시작
#define IAHRS_OBJ_SETW_GYRO_MAX_SCALE "gs"                       // 자이로 센서 최대 측정 스케일 설정 (0~250dps, 1~500dps, 2~1000dps, 3~2000dps)
#define IAHRS_OBJ_SETW_GYRO_LPF_CUTOFF_FREQ "gl"                 // 자이로 센서 내장 저역 통과 필터 차단 주파수 설정 (-1=바이패스, 0~7=LPF cutoff freq.)
#define IAHRS_OBJ_SETW_ACCEL_MAX_SCALE "as"                      // 가속도 센서 최대 측정 스케일 설정 (0~2g, 1~4g, 2~8g, 3~16g)
#define IAHRS_OBJ_SETW_ACCEL_LPF_CUTOFF_FREQ "al"                // 가속도 센서 내장 저역 통과 필터 차단 주파수 설정 (-1=바이패스, 0~7=LPF cutoff freq.)
#define IAHRS_OBJ_SETW_ANGULAR_DRIFT_LIMIT "dt"                  // 각속도의 드리프트 감소 필터 작동 임계값 (0~360)
#define IAHRS_OBJ_SETW_ANGULAR_DRIFT_ERROR "dg"                  // 각속도의 드리프트 감소 필터 오차에 대한 이득 (0~360)
#define IAHRS_OBJ_SETW_ACCEL_SCALE_ADJUST_LIMIT "st"             // 가속도의 스케일 조정 필터 작동 임계값 (0~1)
#define IAHRS_OBJ_SETW_ACCEL_SCALE_ADJUST_ERROR "sg"             // 가속도의 스케일 조정 필터 오차에 대한 이득 (0~1)
#define IAHRS_OBJ_SETW_YAW_KALMAN_FILTER_VARIANCE "av"           // 회전 Kalman filter의 자세 보정에 대한 분산 (0.001~1000, 0=disable)
#define IAHRS_OBJ_SETW_PITCH_KALMAN_FILTER_VARIANCE "mv"         // 회전 Kalman filter의 방위 보정에 대한 분산 (0.001~1000, 0=disable)
#define IAHRS_OBJ_SETW_ZERO_VELOCITY_KALMAN_FILTER_VARIANCE "zv" // 속도 Kalman filter의 영속도 보정에 대한 분산 (0.001~1000, 0=disable)

typedef enum {
  IAHRS_DRIVER_SYNC_FLAG_NONE = 0x00,

  IAHRS_DRIVER_SYNC_FLAG_1MS_TIME = 0x0001,
  IAHRS_DRIVER_SYNC_FLAG_TEMP = 0x0002,
  IAHRS_DRIVER_SYNC_FLAG_SENSOR_ACCEL = 0x0004,
  IAHRS_DRIVER_SYNC_FLAG_SENSOR_GYRO = 0x0008,
  IAHRS_DRIVER_SYNC_FLAG_SENSOR_MAG = 0x0010,
  IAHRS_DRIVER_SYNC_FLAG_GRAVITY_REMOVED_ACCEL = 0x0020,
  IAHRS_DRIVER_SYNC_FLAG_EULER_ANGLE = 0x0040,
  IAHRS_DRIVER_SYNC_FLAG_QUATERNION = 0x0080,
  IAHRS_DRIVER_SYNC_FLAG_GLOBAL_VELOCITY = 0x0100,
  IAHRS_DRIVER_SYNC_FLAG_GLOBAL_POSITION = 0x0200,
  IAHRS_DRIVER_SYNC_FLAG_VIBRATION = 0x0400,

  IAHRS_DRIVER_SYNC_FLAG_MAX = 0x0800,
} iahrs_driver_sync_flag_t;

// #define IAHRS_SYNC_DATA_1MS_TIME 0x0001         // 1ms count 값 [ms]
// #define IAHRS_SYNC_TEMP 0x0002                  // 온도 [°C]
// #define IAHRS_SYNC_SENSOR_ACCEL 0x0004          // 센서 좌표계 가속도 [g]
// #define IAHRS_SYNC_SENSOR_GYRO 0x0008           // 센서 좌표계 각속도 [deg/s]
// #define IAHRS_SYNC_SENSOR_MAG 0x0010            // 센서 좌표계 지자기 [μT]
// #define IAHRS_SYNC_GRAVITY_REMOVED_ACCEL 0x0020 // 전역 좌표계 중력 제거 가속도 [m/s²]
// #define IAHRS_SYNC_EULER_ANGLE 0x0040           // Euler angle [deg]
// #define IAHRS_SYNC_QUATERNION 0x0080            // Quaternion
// #define IAHRS_SYNC_GLOBAL_VELOCITY 0x0100       // 전역 좌표계 속도 [m/s]
// #define IAHRS_SYNC_GLOBAL_POSITION 0x0200       // 전역 좌표계 위치 [m]
// #define IAHRS_SYNC_VIBRATION 0x0400             // 진동 주파수와 크기 [Hz, m/s or deg/s]

#endif /* INCLUDE_IAHRS_ROS2_DRIVER_IAHRS_OBJ */
