# Modified for FAST-LIVO2-GNSS, September–October 2026; see THIRD_PARTY_NOTICES.md.
# SPDX-License-Identifier: GPL-2.0-only
# Upstream FAST-LIVO2 CMakeLists.txt (hku-mars @0d2c034) plus the GNSS build options and targets below.
cmake_minimum_required(VERSION 2.8.3)
project(fast_livo C CXX)   # C: third_party/rtklib (DESIGN §12.1)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Specific settings for Release build
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 64-bit ARM optimizations (e.g., RK3588 and Jetson Orin NX)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
    message(STATUS "Using 64-bit ARM optimizations: -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    # 32-bit ARM optimizations with NEON support
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
    message(STATUS "Using 32-bit ARM optimizations: -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  add_definitions(-DARM_ARCH)
else()
  # x86-64 (Intel/AMD) optimizations
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops") #-flto
  message(STATUS "Using general x86 optimizations: -O3 -march=native -mtune=native -funroll-loops") 
  add_definitions(-DX86_ARCH)
endif()


# ---- GNSS tight-coupling build options (IMPL_PLAN 0/2). Defaults reproduce the upstream compile commands
# of every pre-existing target bit for bit (this is the precondition of the bitwise gates, DESIGN §13):
#   FASTLIVO_OMP=ON        -> -DMP_EN exactly as upstream (OFF = eval profile, no MP_EN; -fopenmp stays for omp_get_wtime)
#   FASTLIVO_STRICT_FP=OFF -> nothing added (ON = every TU -ffp-contract=off -DEIGEN_DONT_PARALLELIZE)
#   FASTLIVO_WITH_GNSS=ON  -> adds third_party/rtklib, gnss, gnss_io, livo_gnss_ros, fastlivo_replay; never touches
#                             the flags/definitions of the upstream targets (only new include paths appear on new targets)
option(FASTLIVO_OMP "Define MP_EN for the OpenMP loops (voxel_map.cpp:660, vio.cpp:1554)" ON)
option(FASTLIVO_STRICT_FP "eval profile: -ffp-contract=off and EIGEN_DONT_PARALLELIZE for all TUs" OFF)
option(FASTLIVO_WITH_GNSS "Build the GNSS core, I/O, ROS glue and the replay driver" ON)
# Since P3 the LIVMapper hooks include the glue (OWNERSHIP §2b): the option is fixed ON and the zero channel is the
# run-time switch gnss.enable = false (bit-identical to ref-R, G-零通道). INTERFACE_CHANGES P3-B #4.
if(NOT FASTLIVO_WITH_GNSS)
  message(FATAL_ERROR "FASTLIVO_WITH_GNSS=OFF is not supported since the P3 hooks: LIVMapper.cpp needs the GNSS glue. "
                      "Build with the default ON and set gnss.enable: false for the upstream behaviour.")
endif()
if(FASTLIVO_STRICT_FP)
  add_compile_options(-ffp-contract=off)
  add_definitions(-DEIGEN_DONT_PARALLELIZE)
endif()

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  if(FASTLIVO_OMP)
    add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
    message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=${PROC_NUM})
    message(STATUS "FASTLIVO_OMP=OFF: MP_EN not defined (eval profile). Cores: ${PROC_NUM}")
  endif()
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  if(FASTLIVO_OMP)
    add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
    message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=${PROC_NUM})
    message(STATUS "FASTLIVO_OMP=OFF: MP_EN not defined (eval profile). Cores: ${PROC_NUM}")
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# Check for OpenMP support
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# Find catkin and required dependencies
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
  vikit_common
  vikit_ros
  cv_bridge
  image_transport
  rosbag
  topic_tools
  diagnostic_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

set(Sophus_LIBRARIES libSophus.so)

# Define the catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime cv_bridge vikit_common vikit_ros image_transport rosbag topic_tools diagnostic_msgs
  DEPENDS EIGEN3 PCL OpenCV Sophus
)

# Include directories for dependencies
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  include
)

# Add libraries
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)

# Add the main executable
add_executable(fastlivo_mapping src/main.cpp)

# Link libraries to the executable
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Link mimalloc if found
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

# AMENDMENTS A28 (eval profile only): EIGEN_DONT_PARALLELIZE stops Eigen/Core from including <omp.h>, and upstream TUs
# (IMU_Processing.cpp, preprocess.cpp) call omp_get_wtime() without including it. Force the declaration into the
# upstream targets; no OpenMP flag, definition or runtime parallelism changes. Default profile: nothing added.
if(FASTLIVO_STRICT_FP)
  foreach(upstream_target vio lio pre imu_proc laser_mapping fastlivo_mapping)
    target_compile_options(${upstream_target} PRIVATE "SHELL:-include omp.h")
  endforeach()
endif()

# ---------------------------------------------------------------------------------------------------
# GNSS tight coupling (IMPL_PLAN 2/5/6/7). Everything below only *adds* targets. The upstream targets
# above keep their exact flags/definitions (compile_commands gate: only new -I paths on new targets).
# ---------------------------------------------------------------------------------------------------
if(FASTLIVO_WITH_GNSS)
  # Each subdirectory resets the inherited include/definition/option directory properties so the
  # ROS-free libraries compile identically here and in `cmake -S gnss`.
  add_subdirectory(third_party/rtklib)   # livo_rtklib (BSD-2, static, hidden)
  add_subdirectory(gnss)                 # livo_gnss (SHARED) + livo_gnss_static + livo_gnss_headers
  add_subdirectory(gnss_io)              # livo_gnss_io (static, PIC)

  # ROS1 glue (GPL-2.0-only): StatesGroup<->CoreState view, subscriptions/replay dispatch, outputs.
  # include/IMU_Processing.h:22 defines a non-inline time_list() (m1: it is NOT in common_lib.h); upstream
  # keeps exactly one TU that includes IMU_Processing.h / LIVMapper.h per library (imu_proc, laser_mapping).
  # The same rule applies here: livo_host_view.cpp and gnss_ros_bridge.cpp both include IMU_Processing.h, so
  # they must live in different libraries; outputs.cpp includes neither.
  add_library(livo_host_view ros/src/livo_host_view.cpp)
  target_include_directories(livo_host_view PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/ros/include)
  target_link_libraries(livo_host_view livo_gnss ${catkin_LIBRARIES})
  add_library(livo_gnss_ros ros/src/gnss_ros_bridge.cpp ros/src/outputs.cpp)
  target_include_directories(livo_gnss_ros PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/ros/include)
  target_link_libraries(livo_gnss_ros
    livo_host_view
    livo_gnss
    livo_gnss_io
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Sophus_LIBRARIES}
  )
  add_dependencies(livo_gnss_ros livo_host_view livo_gnss livo_gnss_io)

  # P3 hooks (INTERFACE_CHANGES P3-B #1): LIVMapper.cpp owns a GnssRosBridge, so laser_mapping links the glue.
  # NO_SYSTEM_FROM_IMPORTED keeps the include flags of LIVMapper.cpp as upstream wrote them: without it the imported
  # Eigen3/OpenCV targets reached through livo_gnss_ros turn its -I/usr/include/eigen3 and -I/usr/include/opencv4 into
  # trailing -isystem flags and the compile-command gate fails; with it only ros/include, gnss/include, gnss_io/include
  # are appended.
  target_link_libraries(laser_mapping livo_gnss_ros)
  set_target_properties(laser_mapping PROPERTIES NO_SYSTEM_FROM_IMPORTED ON)

  # P3-B #2: the glue and hook tests with I14 (rostest, TEST_PLAN I14 / AMENDMENTS A20). Built by `catkin_make tests`,
  # not by `all`, so neither the --ros build nor the compile-command gate sees it. The test TU uses Eigen's OpenMP GEMM
  # (-fopenmp is only a compile option upstream), hence the OpenMP runtime on its link line.
  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(test_p3b_glue ros/test/p3b_glue.test ros/test/test_p3b_glue.cpp)
    target_link_libraries(test_p3b_glue livo_gnss_ros livo_host_view livo_gnss livo_gnss_io imu_proc
      ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_CXX_LIBRARIES})
    target_compile_definitions(test_p3b_glue PRIVATE LIVO_GNSS_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/gnss/test/data")
  endif()

  # Deterministic offline replay driver (DESIGN §13). Links the same estimator libraries as fastlivo_mapping.
  add_executable(fastlivo_replay src/replay_main.cpp)
  target_link_libraries(fastlivo_replay
    laser_mapping
    vio
    lio
    pre
    imu_proc
    livo_gnss_ros
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  if(mimalloc_FOUND)
    target_link_libraries(fastlivo_replay mimalloc)
  endif()
endif()
