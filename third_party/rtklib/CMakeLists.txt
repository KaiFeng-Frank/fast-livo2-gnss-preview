# SPDX-License-Identifier: BSD-2-Clause
# livo_rtklib — RTKLIB-EX v2.5.1 的最小静态库(DESIGN §12.1)。
#
# 为什么不用上游 src/CMakeLists.txt:上游把全部 .c(含 SOFA 派生的 sofa.c、各厂商解码器)编成 SHARED,
# 并覆盖 CMAKE_C_FLAGS;本库只要紧耦合用到的闭包,且要求 -O2 -fno-fast-math -ffp-contract=off(逐位可复现)。
#
# 补丁机制:配置期把 upstream/src 复制到构建树 ${RTKLIB_PATCHED_DIR}/src,再按 patches/*.patch 的文件名顺序
# 用 `patch -p1` 应用(补丁相对 upstream/ 根生成,即路径形如 a/src/rtklib.h)。原样目录 upstream/ 永不改动。
# 复制+打补丁只在「原样文件、补丁集、桩、开关」的联合 sha256 变化时重做(stamp 文件),否则每次配置都会
# 刷新时间戳、让 --ros 的每次 catkin_make --force-cmake 重编整个库(m18)。
#
# LIVO_RTKLIB_APPLY_PATCHES=OFF 编出未打补丁的库(U5 的 pntpos 金标准由这样的一次构建生成,m17);
# 默认 ON。两份不能链进同一二进制,所以对比走金标准文件。
cmake_minimum_required(VERSION 3.10)
project(livo_rtklib C)

option(LIVO_RTKLIB_APPLY_PATCHES "Apply patches/*.patch to the build-tree copy of upstream/src" ON)

set(RTKLIB_UPSTREAM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/upstream")
set(RTKLIB_PATCH_DIR    "${CMAKE_CURRENT_SOURCE_DIR}/patches")
set(RTKLIB_PATCHED_DIR  "${CMAKE_CURRENT_BINARY_DIR}/patched")
set(RTKLIB_STAMP        "${CMAKE_CURRENT_BINARY_DIR}/patched.stamp")

# ---- 1. 复制原样源码 + 顺序打补丁(配置期;任何补丁或原样文件改动都触发重新配置与重做) ----
file(GLOB RTKLIB_PATCHES "${RTKLIB_PATCH_DIR}/*.patch")
list(SORT RTKLIB_PATCHES)
file(GLOB_RECURSE RTKLIB_UPSTREAM_FILES "${RTKLIB_UPSTREAM_DIR}/src/*")
list(SORT RTKLIB_UPSTREAM_FILES)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${RTKLIB_PATCHES} ${RTKLIB_UPSTREAM_FILES})

set(_rtklib_hash_input "apply=${LIVO_RTKLIB_APPLY_PATCHES}\n")
foreach(f ${RTKLIB_UPSTREAM_FILES} ${RTKLIB_PATCHES} "${CMAKE_CURRENT_SOURCE_DIR}/rtklib_stubs.c")
  file(SHA256 "${f}" _h)
  file(RELATIVE_PATH _rel "${CMAKE_CURRENT_SOURCE_DIR}" "${f}")
  string(APPEND _rtklib_hash_input "${_h}  ${_rel}\n")
endforeach()
string(SHA256 RTKLIB_TREE_HASH "${_rtklib_hash_input}")
set(_have "")
if(EXISTS "${RTKLIB_STAMP}")
  file(READ "${RTKLIB_STAMP}" _have)
  string(STRIP "${_have}" _have)
endif()

if(NOT _have STREQUAL RTKLIB_TREE_HASH OR NOT EXISTS "${RTKLIB_PATCHED_DIR}/src/rtklib.h")
  file(REMOVE_RECURSE "${RTKLIB_PATCHED_DIR}")
  file(MAKE_DIRECTORY "${RTKLIB_PATCHED_DIR}")
  file(COPY "${RTKLIB_UPSTREAM_DIR}/src" DESTINATION "${RTKLIB_PATCHED_DIR}")
  # sofa.c:IAU SOFA 派生、许可证不同,从构建树里删掉,任何 glob 都碰不到它
  file(REMOVE "${RTKLIB_PATCHED_DIR}/src/sofa.c")

  if(LIVO_RTKLIB_APPLY_PATCHES)
    find_program(PATCH_EXECUTABLE patch)
    foreach(p ${RTKLIB_PATCHES})
      if(NOT PATCH_EXECUTABLE)
        message(FATAL_ERROR "livo_rtklib: patches present but `patch` not found")
      endif()
      get_filename_component(pname "${p}" NAME)
      execute_process(
        COMMAND "${PATCH_EXECUTABLE}" -p1 --no-backup-if-mismatch -s -i "${p}"
        WORKING_DIRECTORY "${RTKLIB_PATCHED_DIR}"
        RESULT_VARIABLE prc OUTPUT_VARIABLE pout ERROR_VARIABLE perr)
      if(NOT prc EQUAL 0)
        file(REMOVE "${RTKLIB_STAMP}")
        message(FATAL_ERROR "livo_rtklib: applying ${pname} failed:\n${pout}\n${perr}")
      endif()
      message(STATUS "livo_rtklib: applied ${pname}")
    endforeach()
  else()
    message(STATUS "livo_rtklib: LIVO_RTKLIB_APPLY_PATCHES=OFF (unpatched reference build for the U5 golden file)")
  endif()
  file(WRITE "${RTKLIB_STAMP}" "${RTKLIB_TREE_HASH}\n")
else()
  message(STATUS "livo_rtklib: patched tree up to date (stamp ${RTKLIB_TREE_HASH})")
endif()

# ---- 2. 源文件集合:紧耦合闭包(satposs/pntpos/rinex/ublox 及其依赖;链接实测补齐) ----
# rcvraw.c 的 input_raw() 分发到全部厂商解码器,所以 rcv/ 下的文件必须整体进来(全部 BSD-2);
# 未链接进最终可执行文件的目标文件由静态库按需丢弃。
set(RTKLIB_CORE_SOURCES
  src/rtkcmn.c src/ephemeris.c src/pntpos.c src/rinex.c src/ionex.c src/preceph.c src/sbas.c
  src/rcvraw.c src/rcv/ublox.c
  # 由链接闭包补齐:rcvraw.c 的 input_raw() 分发到下列解码器(与上游 src/CMakeLists.txt 的 rcv 集合一致);
  # rcv/comnav.c 在 v2.5.1 里已失修(lam_carr/nav.leaps 不存在,上游 CMake 也不编它)、rcv/tersus.c 上游同样不编,
  # 二者的 input_cnav*/input_tersus* 由 rtklib_stubs.c 桩掉(返回 0 = 无消息)。
  src/rcv/binex.c src/rcv/crescent.c src/rcv/javad.c src/rcv/novatel.c src/rcv/nvs.c
  src/rcv/rt17.c src/rcv/septentrio.c src/rcv/skytraq.c src/rcv/swiftnav.c src/rcv/unicore.c
  src/rtcm.c src/rtcm2.c src/rtcm3.c src/rtcm3e.c src/trace.c
)
set(RTKLIB_SOURCES)
foreach(s ${RTKLIB_CORE_SOURCES})
  list(APPEND RTKLIB_SOURCES "${RTKLIB_PATCHED_DIR}/${s}")
endforeach()
list(APPEND RTKLIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/rtklib_stubs.c")

# 作为子目录时不继承宿主的 include/定义/选项(它们是 C++/ROS 的东西,会污染逐位可复现的 C 编译命令)。
# 必须在 add_library 之前:目标的 INCLUDE_DIRECTORIES/COMPILE_* 在创建时从目录属性初始化。
set_property(DIRECTORY PROPERTY INCLUDE_DIRECTORIES "")
set_property(DIRECTORY PROPERTY COMPILE_DEFINITIONS "")
set_property(DIRECTORY PROPERTY COMPILE_OPTIONS "")

add_library(livo_rtklib STATIC ${RTKLIB_SOURCES})
add_library(livo::rtklib ALIAS livo_rtklib)

# INTERFACE 包含路径与宏:消费者(gnss/、gnss_io/)不把它们挂到目标上,而是用 set_source_files_properties 只
# 交给允许包含 rtklib.h 的那几个 TU(m14);本目标自身的 TU 通过 PUBLIC 得到同一套。
target_include_directories(livo_rtklib PUBLIC "${RTKLIB_PATCHED_DIR}/src")

# ---- 3. PUBLIC 宏:obsd_t/nav_t 布局依赖(rtklib.h:158-165, :205-216),adapter TU 必须看到同一套 ----
target_compile_definitions(livo_rtklib PUBLIC ENAGLO ENAQZS ENACMP ENAGAL ENAIRN NFREQ=3 NEXOBS=3)
# MAXPRNCMP=63 只有在 P1 补丁给 rtklib.h:211 加了 #ifndef 之后才能从命令行设定;否则头文件会重定义回 50
# (值仍全 TU 一致,但产生 redefinition 警告)。P1 落地(patches/P1*.patch 出现)且补丁开启即自动开启。
file(GLOB RTKLIB_P1 "${RTKLIB_PATCH_DIR}/P1*.patch")
if(RTKLIB_P1 AND LIVO_RTKLIB_APPLY_PATCHES)
  target_compile_definitions(livo_rtklib PUBLIC MAXPRNCMP=63)
  message(STATUS "livo_rtklib: MAXPRNCMP=63 (P1 present)")
else()
  message(STATUS "livo_rtklib: MAXPRNCMP left at upstream 50 until patches/P1*.patch lands (C51-C63 -> satno()==0)")
endif()
# 不定义 TRACE(DESIGN §12.1);不定义 WIN32/DLL。

# ---- 4. 编译选项:-O2 -fno-fast-math -ffp-contract=off -w;独立于宿主 profile,永远逐位可复现 ----
set(CMAKE_C_FLAGS "")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g")
target_compile_options(livo_rtklib PRIVATE -O2 -fno-fast-math -ffp-contract=off -w -fno-strict-aliasing)
set_target_properties(livo_rtklib PROPERTIES
  POSITION_INDEPENDENT_CODE ON      # 会被链进 SHARED 的 livo_gnss
  C_VISIBILITY_PRESET hidden        # 符号只在 livo_gnss 内部可见;再加 --exclude-libs 双保险
)
target_link_libraries(livo_rtklib PUBLIC m pthread)
