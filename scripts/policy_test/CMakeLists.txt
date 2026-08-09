cmake_minimum_required(VERSION 3.16)
project(policy_test LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# n100_cpp/ is a verbatim mirror of IMU_N100_Test/src/cpp_n100. Nothing inside it
# is edited here, so it can be refreshed with rsync whenever the IMU repo moves
# on; the Python module is built alongside it rather than from within it.
add_subdirectory(n100_cpp)

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

# The static library from n100_cpp already owns the target name `n100`, so the
# extension module is built under a different target and renamed on output.
pybind11_add_module(n100_python n100_binding.cpp)
target_link_libraries(n100_python PRIVATE n100::n100)
target_compile_options(n100_python PRIVATE -Wall -Wextra -Wpedantic)

# Drop the .so next to print_policy_values.py so the scripts stay runnable the
# way the rest of this repo is: `cd scripts/policy_test && python3 print_policy_values.py`.
set_target_properties(n100_python PROPERTIES
  OUTPUT_NAME n100
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
