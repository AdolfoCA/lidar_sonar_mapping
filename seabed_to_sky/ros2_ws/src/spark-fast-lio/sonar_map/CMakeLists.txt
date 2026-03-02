cmake_minimum_required(VERSION 3.5)
project(sonar_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python executables - rename to remove .py extension
install(PROGRAMS src/sonar_scan_ned.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sonar_scan_ned
)

install(PROGRAMS src/sonar_map_ned.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sonar_map_ned
)

install(PROGRAMS src/save_map.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME save_map
)

# norbit mapping 
install(PROGRAMS src/norbit_map_ned.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME norbit_map_ned
)

install(PROGRAMS src/norbit_scan_ned.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME norbit_scan_ned
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()