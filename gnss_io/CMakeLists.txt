# SPDX-License-Identifier: BSD-3-Clause
# livo_gnss_io — ROS-free I/O: ROS1 wire decoder driven by message_definition, gnss_comm field mapping,
# RINEX (readrnxt) and UBX (input_ubx) sources (DESIGN §12.2–§12.4). Pure C++17; depends on Eigen (via the
# livo_gnss public headers) and livo_rtklib (rinex/ubx only). Static, PIC (linked into the SHARED ROS glue).
#
# Its tests (U2, T1–T5) are normally built through gnss/CMakeLists.txt (LIVO_GNSS_BUILD_TESTS, same options,
# one ctest); the standalone build below exists for a quick local check and uses the same flags (M7).
cmake_minimum_required(VERSION 3.11)
project(livo_gnss_io CXX C)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(LIVO_GNSS_IO_STANDALONE ON)
else()
  set(LIVO_GNSS_IO_STANDALONE OFF)
endif()
option(LIVO_GNSS_IO_BUILD_TESTS "Build gnss_io gtest targets" ${LIVO_GNSS_IO_STANDALONE})
option(LIVO_GNSS_STRICT_FP "-ffp-contract=off (also implied by FASTLIVO_STRICT_FP)" OFF)

if(LIVO_GNSS_IO_STANDALONE)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  # Identical to gnss/CMakeLists.txt's standalone block (same -march/-pthread as the catkin build, M7).
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -mcpu=native -mtune=native")
  else()
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -O3 -march=native -mtune=native -funroll-loops")
  endif()
endif()

# Do not inherit the host's include dirs / definitions / options (keeps the library ROS-free by construction).
set_property(DIRECTORY PROPERTY INCLUDE_DIRECTORIES "")
set_property(DIRECTORY PROPERTY COMPILE_DEFINITIONS "")
set_property(DIRECTORY PROPERTY COMPILE_OPTIONS "")

find_package(Eigen3 REQUIRED)
if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
endif()

if(NOT TARGET livo_rtklib)
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../third_party/rtklib" "${CMAKE_CURRENT_BINARY_DIR}/rtklib")
endif()
# livo_gnss public headers (types only; no link dependency on the core from this library).
if(NOT TARGET livo_gnss_headers)
  add_library(livo_gnss_headers INTERFACE)
  target_include_directories(livo_gnss_headers INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../gnss/include")
  target_link_libraries(livo_gnss_headers INTERFACE Eigen3::Eigen)
endif()

set(LIVO_GNSS_IO_COMMON_OPTIONS -Wall -Wextra -Wno-unused-parameter)
if(LIVO_GNSS_STRICT_FP OR FASTLIVO_STRICT_FP)
  list(APPEND LIVO_GNSS_IO_COMMON_OPTIONS -ffp-contract=off)
endif()

add_library(livo_gnss_io STATIC
  src/ros1_msgdef.cpp
  src/ros1_decoder.cpp
  src/gnss_comm_mapping.cpp
  src/rinex_source.cpp     # includes rtklib.h (registered deviation, AMENDMENTS A21)
  src/ubx_stream.cpp       # includes rtklib.h (idem)
)
add_library(livo::gnss_io ALIAS livo_gnss_io)
target_include_directories(livo_gnss_io PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
# LINK_ONLY: consumers link livo_rtklib for the two pimpl TUs' symbols, but neither this target's other TUs
# nor the consumers inherit RTKLIB's include directory / macro set (a PRIVATE link would still hand them to
# every TU of this library; verified on ros1_decoder.cpp's compile command).
target_link_libraries(livo_gnss_io PUBLIC livo_gnss_headers INTERFACE $<LINK_ONLY:livo_rtklib>)
target_compile_features(livo_gnss_io PUBLIC cxx_std_17)
target_compile_definitions(livo_gnss_io PRIVATE EIGEN_DONT_PARALLELIZE)
target_compile_options(livo_gnss_io PRIVATE ${LIVO_GNSS_IO_COMMON_OPTIONS})
set_target_properties(livo_gnss_io PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
# rtklib.h reaches exactly the two pimpl TUs (A21): include dir + macro set attached per source file, as in gnss/.
set_source_files_properties(src/rinex_source.cpp src/ubx_stream.cpp PROPERTIES
  INCLUDE_DIRECTORIES "$<TARGET_PROPERTY:livo_rtklib,INTERFACE_INCLUDE_DIRECTORIES>"
  COMPILE_DEFINITIONS "$<TARGET_PROPERTY:livo_rtklib,INTERFACE_COMPILE_DEFINITIONS>")

if(LIVO_GNSS_IO_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  if(NOT TARGET livo_gnss_static)
    set(LIVO_GNSS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../gnss" "${CMAKE_CURRENT_BINARY_DIR}/gnss")
  endif()
  if(NOT DEFINED LIVO_GNSS_IO_TEST_OPTIONS)
    set(LIVO_GNSS_IO_TEST_OPTIONS ${LIVO_GNSS_IO_COMMON_OPTIONS})
  endif()
  foreach(t test_u02_wire test_t01_t05_io)
    add_executable(${t} test/${t}.cpp)
    target_link_libraries(${t} PRIVATE livo_gnss_io livo_gnss_static GTest::GTest GTest::Main)
    target_compile_definitions(${t} PRIVATE EIGEN_DONT_PARALLELIZE LIVO_GNSS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../gnss/test/data")
    target_compile_options(${t} PRIVATE ${LIVO_GNSS_IO_TEST_OPTIONS})
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()
