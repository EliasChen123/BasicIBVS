#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)


###################################################
#                                                 #
#       Start project customization section       #
#                                                 #
###################################################

PROJECT(remote_surgical_robot)

##################################
#project config
include(./cmake/rtt_macros.cmake)

#rtt config
find_package(PkgConfig REQUIRED)

find_library(
    LIBSERIAL_LIBRARY 
    NAMES serial 
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu 
    NO_DEFAULT_PATH
    REQUIRED)

find_path(LIBSERIAL_INCLUDE_DIR NAMES libserial/SerialPort.h PATHS /usr/include REQUIRED)

link_directories(${LIBSERIAL_INCLUDE_DIR})

find_file(RTT_PC_FILE NAMES orocos-rtt-gnulinux.pc PATHS /usr/local/lib/pkgconfig NO_DEFAULT_PATH)
pkg_check_modules(RTT REQUIRED ${RTT_PC_FILE})

include_directories(${RTT_INCLUDE_DIRS})
link_directories(${RTT_LIBRARY_DIRS})
add_definitions(${RTT_CFLAGS_OTHER})

find_file(KDL_PC_FILE NAMES orocos_kdl.pc PATHS /usr/local/lib/pkgconfig NO_DEFAULT_PATH)
pkg_check_modules(KDL REQUIRED ${KDL_PC_FILE})

include_directories(${KDL_INCLUDE_DIRS})
link_directories(${KDL_LIBRARY_DIRS})
add_definitions(${KDL_CFLAGS_OTHER})

# find_package(glog 0.4.0 REQUIRED)
include_directories(/usr/local/include)

find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

SET(ENV{GLOBAL_LIBRARY_SRCS})
FILE( GLOB CPPS [^.]*.cpp )

# add_subdirectory(commu_test)

add_subdirectory(commu)
add_subdirectory(thread)
add_subdirectory(motor_control)
add_subdirectory(robot)
add_subdirectory(data_pool)

add_executable(main main.cpp $ENV{GLOBAL_LIBRARY_SRCS})

target_link_libraries(main 
      ${RTT_LIBRARIES}
      ${KDL_LIBRARIES}
      ${LIBSERIAL_LIBRARY}
      Eigen3::Eigen
)



