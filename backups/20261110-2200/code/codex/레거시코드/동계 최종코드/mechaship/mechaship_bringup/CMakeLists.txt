cmake_minimum_required(VERSION 3.8)
project(mechaship_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 1. 기존 디렉토리(launch, param) 설치
install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

# 2. [추가] 파이썬 스크립트를 실행 파일로 설치
# scripts 폴더 내의 imu_reformatter.py를 lib/mechaship_bringup 폴더로 복사합니다.
install(PROGRAMS
  scripts/imu_reformatter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()