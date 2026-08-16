cmake_minimum_required(VERSION 3.16)
project(n100_sdk VERSION 0.1.0 LANGUAGES CXX)

# Standalone project: no ROS, no ament, no colcon.
option(N100_BUILD_EXAMPLES "Build the example programs" ON)
option(N100_BUILD_TESTS "Build the hardware free tests" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)

add_library(n100
  src/crc.cpp
  src/serial_port.cpp
  src/imu_driver.cpp
)
add_library(n100::n100 ALIAS n100)

target_include_directories(n100 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(n100 PUBLIC cxx_std_17)
target_link_libraries(n100 PUBLIC Threads::Threads)
target_compile_options(n100 PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(n100 PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(N100_BUILD_EXAMPLES)
  foreach(example read_imu rl_observation)
    add_executable(${example} examples/${example}.cpp)
    target_link_libraries(${example} PRIVATE n100)
    target_compile_options(${example} PRIVATE -Wall -Wextra -Wpedantic)
  endforeach()
endif()

if(N100_BUILD_TESTS)
  enable_testing()
  add_executable(protocol_test tests/protocol_test.cpp)
  target_link_libraries(protocol_test PRIVATE n100)
  target_compile_options(protocol_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME protocol_test COMMAND protocol_test)
endif()

include(GNUInstallDirs)
install(TARGETS n100 EXPORT n100Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT n100Targets
  FILE n100Config.cmake
  NAMESPACE n100::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/n100
)
