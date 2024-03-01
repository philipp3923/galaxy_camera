cmake_minimum_required(VERSION 3.8)
project(mercury_camera)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_NAME entry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-Dev)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dv_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_library(GXIAPI_LIB gxiapi PATHS /usr/lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/include)

add_executable(${EXECUTABLE_NAME}
        src/main.cpp
)

target_link_libraries(
        ${EXECUTABLE_NAME}
        ${GXIAPI_LIB}
        ${OpenCV_LIBS}
)

install(TARGETS ${EXECUTABLE_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
