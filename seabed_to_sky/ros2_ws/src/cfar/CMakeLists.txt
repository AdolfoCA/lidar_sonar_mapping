cmake_minimum_required(VERSION 3.5)
project(cfar)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Python libraries
find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
link_directories(${PYTHON_LIBRARIES})

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Ensure PYTHON_INCLUDE_DIRS is set
set(PYTHON_INCLUDE_DIRS /usr/include/python3.10)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include ${rclcpp_INCLUDE_DIRS})

# Include Pybind11 directories
include_directories(${pybind11_INCLUDE_DIRS})

# Build the C++ shared library for Python
pybind11_add_module(cfar_cpp src/cfar.cpp)
pybind11_add_module(match_cpp src/match.cpp)

# Ensure the library has the proper extension for Python modules
set_target_properties(cfar_cpp PROPERTIES PREFIX "" SUFFIX ".so")
set_target_properties(match_cpp PROPERTIES PREFIX "" SUFFIX ".so")

# Install the library to a proper destination
install(TARGETS cfar_cpp match_cpp
  LIBRARY DESTINATION lib/python3.10/site-packages)

# Install Python package
ament_python_install_package(cfar)

# Install the Python scripts
install(PROGRAMS
  cfar/CFAR.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(ament_cmake pybind11)

# Install the C++ library
install(TARGETS cfar_cpp
    LIBRARY DESTINATION lib)

ament_package()
