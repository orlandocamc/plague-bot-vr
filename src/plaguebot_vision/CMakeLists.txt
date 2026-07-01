cmake_minimum_required(VERSION 3.8)
project(plaguebot_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch web
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_realsense.py
  scripts/camera_kiyo.py
  scripts/camera_webcam.py
  scripts/hailo_detector.py
  scripts/vision_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
