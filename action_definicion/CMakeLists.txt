cmake_minimum_required(VERSION 3.8)
project(action_definicion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)  

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Angulo.action" 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter for this example (adjust as necessary)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

