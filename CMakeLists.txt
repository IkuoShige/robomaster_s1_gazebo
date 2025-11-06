cmake_minimum_required(VERSION 3.8)
project(robomaster_s1_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install urdf files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install worlds
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install documentation
install(DIRECTORY
  docs
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install production scripts only (dev scripts are not installed)
install(PROGRAMS
  scripts/mecanum_teleop_keyboard.py
  scripts/cleanup_gazebo.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
