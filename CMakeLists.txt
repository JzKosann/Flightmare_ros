cmake_minimum_required(VERSION 3.0.2)
project(acados_nmpc_controller)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(message_generation REQUIRED)

# add_message_files( FILES quad_thrusts.msg)
# generate_messages( DEPENDENCIES std_msgs)

catkin_simple()
# ---------------优化
# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

# Setup Architcture-specific Flags
if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
    message(STATUS "Using ARMv7 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM aarch64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else ()
    set(CMAKE_CXX_ARCH_FLAGS " -march=native")
endif ()

# Setup General C++ Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")
# otherwise double free or corruption (out) error when running racing or motion_planning example
add_compile_options(-O3)

# Setup Release and Debug flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} -Wall -DNDEBUG -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -g")
# ---------------优化

catkin_package(
LIBRARIES
CATKIN_DEPENDS
)

cs_add_executable(acados_racing
    src/acados_racing/acados_racing_node.cpp
)
# include_directories(
#     ${catkin_INCLUDE_DIRS}
#     ${catkin_INCLUDE_DIRS}/include
# )

target_link_libraries(acados_racing
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
stdc++fs
zmq
zmqpp
)


catkin_install_python(PROGRAMS
  scripts/ros_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# 安装贴图
install(DIRECTORY asset/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/asset)
