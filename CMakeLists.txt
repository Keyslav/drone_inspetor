cmake_minimum_required(VERSION 3.8)
project(drone_inspetor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)

find_package(cv_bridge REQUIRED)

install(DIRECTORY
  drone_inspetor/launch
  drone_inspetor/config
  drone_inspetor/assets
  drone_inspetor/models/
  drone_inspetor/gui/
  drone_inspetor/nodes/
  drone_inspetor/scripts/
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  drone_inspetor/gui/map.html
  DESTINATION share/${PROJECT_NAME}/gui
)

ament_python_install_package()



