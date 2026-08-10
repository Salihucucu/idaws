cmake_minimum_required(VERSION 3.8)
project(idaws_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config scripts sim
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
