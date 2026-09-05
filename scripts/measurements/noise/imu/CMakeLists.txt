cmake_minimum_required(VERSION 3.16)
project(imu_noise_binding LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(IMU_N100_TEST_ROOT "" CACHE PATH "IMU_N100_Test checkout")
if(NOT IMU_N100_TEST_ROOT AND DEFINED ENV{IMU_N100_TEST_ROOT})
  set(IMU_N100_TEST_ROOT "$ENV{IMU_N100_TEST_ROOT}")
endif()
if(NOT IMU_N100_TEST_ROOT)
  get_filename_component(IMU_N100_TEST_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../IMU_N100_Test" ABSOLUTE)
endif()
set(N100_CPP_DIR "${IMU_N100_TEST_ROOT}/src/cpp_n100")
if(NOT EXISTS "${N100_CPP_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "IMU_N100_Test SDK not found: ${N100_CPP_DIR}")
endif()
add_subdirectory("${N100_CPP_DIR}" "${CMAKE_BINARY_DIR}/n100_cpp")

# Development.Module gives the headers pybind11 3.x needs to create the
# extension target; it does not require linking against libpython.
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)

# Locate pybind11 through the interpreter that is actually going to import the
# module, so an activated .venv wins over anything installed system wide.
if(NOT pybind11_DIR)
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE PYBIND11_QUERY_RESULT
    ERROR_QUIET)
  if(NOT PYBIND11_QUERY_RESULT EQUAL 0)
    message(FATAL_ERROR
      "pybind11 not found for ${Python3_EXECUTABLE}.\n"
      "Activate the venv and run: pip install pybind11")
  endif()
  set(pybind11_DIR "${PYBIND11_CMAKE_DIR}")
endif()
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(n100_python n100_binding.cpp)
target_link_libraries(n100_python PRIVATE n100::n100)
target_compile_options(n100_python PRIVATE -Wall -Wextra -Wpedantic)

set_target_properties(n100_python PROPERTIES
  OUTPUT_NAME n100
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
