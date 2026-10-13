# SPDX-License-Identifier: BSD-3-Clause
# livo_gnss — ROS-free GNSS tight-coupling core (DESIGN §12.5). Builds standalone:
#     cmake -S gnss -B build_core && cmake --build build_core -j2 && (cd build_core && ctest)
# or as a subdirectory of the fast_livo catkin package (top-level CMakeLists.txt).
#
# Targets
#   livo_rtklib        static, hidden (third_party/rtklib)
#   livo_gnss_objs     OBJECT library: all core TUs, -fvisibility=hidden, PIC
#   livo_gnss          SHARED, exports only LIVO_GNSS_API symbols; -Wl,--exclude-libs,liblivo_rtklib.a
#   livo_gnss_static   STATIC from the same objects; tests link this one so they can reach src/internal
#   livo_gnss_sim      STATIC simulator (tests only)
#   test_uNN_*         gtest executables (LIVO_GNSS_BUILD_TESTS); the gnss_io tests (U2, T1–T5) are built and
#                      registered by the same switch with the same options (M7)
cmake_minimum_required(VERSION 3.11)   # 3.11: source-file INCLUDE_DIRECTORIES with generator expressions
project(livo_gnss CXX C)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(LIVO_GNSS_STANDALONE ON)
else()
  set(LIVO_GNSS_STANDALONE OFF)
endif()
option(LIVO_GNSS_BUILD_TESTS "Build the gtest targets (U1–U17 and the gnss_io U2/T1–T5)" ${LIVO_GNSS_STANDALONE})
option(LIVO_GNSS_STRICT_FP "Add -ffp-contract=off (FASTLIVO_STRICT_FP implies it)" OFF)

if(LIVO_GNSS_STANDALONE)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  # Same optimisation/-march as the host package so Eigen fixed-size objects crossing the ABI have the
  # same alignment as in the catkin build (top-level CMakeLists.txt x86 branch; bitwise claims hold only
  # on the same machine + image, DESIGN §13). gnss_io/CMakeLists.txt repeats this block verbatim.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -mcpu=native -mtune=native")
  else()
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -O3 -march=native -mtune=native -funroll-loops")
  endif()
endif()

# Never inherit the host's include dirs / definitions / options (ROOT_DIR, MP_EN, PCL, OpenMP ...):
# the core must compile identically standalone and inside catkin.
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
if(NOT TARGET livo_gnss_headers)
  add_library(livo_gnss_headers INTERFACE)
  target_include_directories(livo_gnss_headers INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
  target_link_libraries(livo_gnss_headers INTERFACE Eigen3::Eigen)
endif()

set(LIVO_GNSS_CORE_SOURCES
  src/types.cpp
  src/core_state.cpp
  src/config.cpp
  src/diagnostics.cpp
  src/time_map.cpp
  src/time_offset.cpp
  src/rtk_adapter.cpp      # the only TU that includes rtklib.h (enforced below)
  src/ephem_store.cpp
  src/quality.cpp
  src/gnss_only.cpp
  src/bridge.cpp
  src/leveling.cpp
  src/aug_block.cpp
  src/rows.cpp
  src/update.cpp
  src/slip.cpp
  src/clock.cpp
  src/integrity.cpp
  src/initializer.cpp
  src/outage.cpp
  src/gnss_fusion.cpp
  src/gnss_fusion_livgo.cpp
  src/livgo_pin.cpp
  src/livgo_init.cpp
)

set(LIVO_GNSS_COMMON_OPTIONS -Wall -Wextra -Wno-unused-parameter)
if(LIVO_GNSS_STRICT_FP OR FASTLIVO_STRICT_FP)
  list(APPEND LIVO_GNSS_COMMON_OPTIONS -ffp-contract=off)
endif()

add_library(livo_gnss_objs OBJECT ${LIVO_GNSS_CORE_SOURCES})
target_include_directories(livo_gnss_objs PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(livo_gnss_objs PUBLIC Eigen3::Eigen)
target_compile_features(livo_gnss_objs PUBLIC cxx_std_17)
# EIGEN_DONT_PARALLELIZE is unconditional in the core: deterministic by construction (DESIGN §13).
target_compile_definitions(livo_gnss_objs PRIVATE LIVO_GNSS_BUILDING EIGEN_DONT_PARALLELIZE)
target_compile_options(livo_gnss_objs PRIVATE ${LIVO_GNSS_COMMON_OPTIONS})
set_target_properties(livo_gnss_objs PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON)
# DESIGN §12.1 "rtklib.h 只在 rtk_adapter.cpp 一个 TU 包含" is enforced by the build, not by convention (m14):
# the RTKLIB include directory and its PUBLIC macro set are attached to that one source file; any other core
# TU that includes rtklib.h fails to compile.
set_source_files_properties(src/rtk_adapter.cpp PROPERTIES
  INCLUDE_DIRECTORIES "$<TARGET_PROPERTY:livo_rtklib,INTERFACE_INCLUDE_DIRECTORIES>"
  COMPILE_DEFINITIONS "$<TARGET_PROPERTY:livo_rtklib,INTERFACE_COMPILE_DEFINITIONS>")

add_library(livo_gnss SHARED $<TARGET_OBJECTS:livo_gnss_objs>)
add_library(livo::gnss ALIAS livo_gnss)
target_include_directories(livo_gnss PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(livo_gnss PUBLIC Eigen3::Eigen PRIVATE livo_rtklib)
target_compile_features(livo_gnss PUBLIC cxx_std_17)
# RTKLIB's unprefixed C symbols (norm, dot, mat, trace, ...) must not leak out of the .so (R3-6).
set_property(TARGET livo_gnss APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--exclude-libs,liblivo_rtklib.a")
set_target_properties(livo_gnss PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)

# Tests link the static variant; it exposes src/internal and (PUBLIC) livo_rtklib so WP-a's U3–U6 test
# files may include rtklib.h (OWNERSHIP §1 note).
add_library(livo_gnss_static STATIC $<TARGET_OBJECTS:livo_gnss_objs>)
target_include_directories(livo_gnss_static PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(livo_gnss_static PUBLIC Eigen3::Eigen livo_rtklib)
target_compile_features(livo_gnss_static PUBLIC cxx_std_17)

if(LIVO_GNSS_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  # gnss_io is needed by U6 and its own tests (U2, T1–T5) are built here with the same options (M7):
  # there is no separate "cmake -S gnss_io" gate any more.
  if(NOT TARGET livo_gnss_io)
    set(LIVO_GNSS_IO_BUILD_TESTS ON CACHE BOOL "" FORCE)
    set(LIVO_GNSS_IO_TEST_OPTIONS ${LIVO_GNSS_COMMON_OPTIONS})
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../gnss_io" "${CMAKE_CURRENT_BINARY_DIR}/gnss_io")
  endif()

  add_library(livo_gnss_sim STATIC sim/simulator.cpp)
  target_include_directories(livo_gnss_sim PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/sim"
                             PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../include")
  target_link_libraries(livo_gnss_sim PUBLIC livo_gnss_static)
  target_compile_definitions(livo_gnss_sim PRIVATE EIGEN_DONT_PARALLELIZE)
  target_compile_options(livo_gnss_sim PRIVATE ${LIVO_GNSS_COMMON_OPTIONS})

  # New test executables: append here (skeleton maintainer, OWNERSHIP §3-3) after the INTERFACE_CHANGES entry.
  set(LIVO_GNSS_TESTS
    test_u01_time test_u03_satmap test_u04_satstate test_u05_corrections test_u06_paths
    test_u07_jacobians test_u08_hooks test_u09_augblock test_u10_updater test_u11_tdcp
    test_u12_gating test_u13_init test_u14_bridge test_u15_mc test_u16_gnss_only test_u17_integrity
    test_wpf_serialisation    # WP-f CSV/JSON/canonicalDump serialisation (A40)
    test_p3_facade            # P3-A2 GnssFusion facade (OWNERSHIP §2c)
    test_livgo)               # FAST-LIVGO arm: DTW lag
  foreach(t ${LIVO_GNSS_TESTS})
    add_executable(${t} test/${t}.cpp)
    target_link_libraries(${t} PRIVATE livo_gnss_static livo_gnss_sim livo_gnss_io GTest::GTest GTest::Main)
    target_compile_definitions(${t} PRIVATE EIGEN_DONT_PARALLELIZE LIVO_GNSS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
    target_compile_options(${t} PRIVATE ${LIVO_GNSS_COMMON_OPTIONS})
    add_test(NAME ${t} COMMAND ${t})
  endforeach()
endif()
