cmake_minimum_required(VERSION 3.16)
project(AgiliciousBetaflightPlugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(AEROLOOP_PLUGIN_SOURCE "" CACHE FILEPATH
    "Path to Aeroloop's BetaflightPlugin.cc")
if(NOT EXISTS "${AEROLOOP_PLUGIN_SOURCE}")
  message(FATAL_ERROR
          "AEROLOOP_PLUGIN_SOURCE is missing: ${AEROLOOP_PLUGIN_SOURCE}")
endif()

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-math7 REQUIRED)
# The patched plugin publishes ground-truth odometry for the outer loop.
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

# Build a patched copy in the build tree; never edit the user's Aeroloop tree.
# The patch fixes the 2026.6 packet sizes and prevents a one-second socket wait
# from blocking Gazebo's physics update thread.
find_program(PATCH_EXECUTABLE patch)
if(NOT PATCH_EXECUTABLE)
  message(FATAL_ERROR "The 'patch' executable is required")
endif()
set(PATCHED_PLUGIN_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/BetaflightPlugin.cc")
configure_file("${AEROLOOP_PLUGIN_SOURCE}" "${PATCHED_PLUGIN_SOURCE}" COPYONLY)
execute_process(
  COMMAND "${PATCH_EXECUTABLE}" --batch --forward --fuzz=0 -p0
          -i "${CMAKE_CURRENT_SOURCE_DIR}/BetaflightPlugin.patch"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  RESULT_VARIABLE PATCH_RESULT
  OUTPUT_QUIET
  ERROR_VARIABLE PATCH_ERROR)
if(NOT PATCH_RESULT EQUAL 0)
  message(FATAL_ERROR "Could not patch Aeroloop BetaflightPlugin: ${PATCH_ERROR}")
endif()

add_library(AgiliciousBetaflightPlugin SHARED "${PATCHED_PLUGIN_SOURCE}")
get_filename_component(AEROLOOP_PLUGIN_DIRECTORY
  "${AEROLOOP_PLUGIN_SOURCE}" DIRECTORY)
target_include_directories(AgiliciousBetaflightPlugin PRIVATE
  "${AEROLOOP_PLUGIN_DIRECTORY}")
target_link_libraries(AgiliciousBetaflightPlugin PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13)

add_library(AgiliciousAerodynamicsPlugin SHARED
  AgiliciousAerodynamicsPlugin.cc
  AerodynamicsModel.cc)
target_link_libraries(AgiliciousAerodynamicsPlugin PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13)

enable_testing()
add_executable(aero_model_test aero_model_test.cc AerodynamicsModel.cc)
add_test(NAME aero_model_test COMMAND aero_model_test)

add_executable(aero_envelope aero_envelope.cc AerodynamicsModel.cc)
