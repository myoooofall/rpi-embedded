cmake_minimum_required(VERSION 3.16)
# set(CMAKE_UNITY_BUILD ON)
# chip option
# option(WIRINGPI_V "wiringpi library" OFF)   # SHIT wiringpi !
option(CM4_VERSION "compile for CM4" ON)
option(ROCKS_VERSION "compile for ROCKS" OFF)
if(ROCKS_VERSION)
    set(CMAKE_C_COMPILER aarch64-none-linux-gnu-gcc)
    set(CMAKE_CXX_COMPILER aarch64-none-linux-gnu-g++)
    set(CMAKE_EXE_LINKER_FLAGS "-static")
endif()

project(S01 LANGUAGES CXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

if(CM4_VERSION)
    set(LIBS pigpio)
elseif(ROCKS_VERSION)
    include_directories(
        $ENV{HOME}/aarch64-toolchains
        $ENV{HOME}/aarch64-toolchains/include
        $ENV{HOME}/aarch64-toolchains/fmt-aarch64
        $ENV{HOME}/aarch64-toolchains/mraa-aarch64/include
    )
    set(LIBS
        $ENV{HOME}/aarch64-toolchains/mraa-aarch64/lib/libmraa.a
    )
endif()

find_package(Threads REQUIRED)
# find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Protobuf REQUIRED)
find_library(RF24 rf24 REQUIRED)
message(STATUS "using RF24 library: ${RF24}")

find_package(zos_core REQUIRED)
message("zos_core_FOUND : " ${zos_core_FOUND})
message("zos_core_INCLUDE_DIRS : " ${zos_core_INCLUDE_DIRS})
message("zos_core_LIBRARIES : " ${zos_core_LIBRARIES})

# detect the CPU make and type
# include(cmake/detectCPU.cmake) # sets the variable SOC accordingly

# auto-detect what driver to use
# auto-detect can be overriden using `cmake .. -D RF24_DRIVER=<supported driver>`
# include(cmake/AutoConfig_RF24_DRIVER.cmake)

include_directories(
    # ${pigpio_INCLUDE_DIRS}
    ${fmt_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/share/proto
    ${CMAKE_CURRENT_SOURCE_DIR}/control
    # ${CMAKE_CURRENT_SOURCE_DIR}/share/zos/project/core/include
    # ${CMAKE_CURRENT_SOURCE_DIR}/share/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/robot
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set(LibMRAA /usr/local/lib/libmraa.so)

set(LIBS
    ${LIBS}
    ${LibMRAA}
    ${Protobuf_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    Threads::Threads
    # fmt::fmt
    ${RF24}
)
file(GLOB SHARED_SOURCE_FILES
    ${SHARED_SOURCE_FILES}
    ${CMAKE_CURRENT_SOURCE_DIR}/share/proto/*.pb.cc
    # ${CMAKE_CURRENT_SOURCE_DIR}/control/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/share/utils/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/robot/robotz.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/robot/wifiz.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/robot/nrf2401.cpp
)
# chip option
if(CM4_VERSION)
    file(GLOB SHARED_SOURCE_FILES
        ${SHARED_SOURCE_FILES}
        # if(WIRINGPI_V)
        #     ${CMAKE_CURRENT_SOURCE_DIR}/robot/device_CM4.cpp
        # endif()
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/device_pigpio.cpp
    )
elseif(ROCKS_VERSION)
    file(GLOB SHARED_SOURCE_FILES
        ${SHARED_SOURCE_FILES}
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/device_ROCKS.cpp
    )
endif()


file(GLOB TEST_FILES
    # ${CMAKE_CURRENT_SOURCE_DIR}/unit_test/test_comm.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/unit_test/test_comm_2401.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit_test/test_old_ver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit_test/test_control.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unit_test/test_i2c.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/unit_test/*.cpp
)

foreach(test_file ${TEST_FILES})
    file(RELATIVE_PATH test_file_name ${CMAKE_CURRENT_SOURCE_DIR} ${test_file})
    string(REPLACE ".cpp" "" test_name ${test_file_name})
    string(REPLACE "test/" "" test_name ${test_name})
    message("test file: " ${test_file} " -- " ${test_name})

    add_executable (${test_name} ${test_file_name} ${SHARED_SOURCE_FILES})
    target_link_libraries (${test_name} ${LIBS} zos::zos_core)
    if(CM4_VERSION)
        target_compile_definitions(${test_name} PRIVATE CM4_VERSION)
    elseif(ROCKS_VERSION)
        target_compile_definitions(${test_name} PRIVATE ROCKPIS_VERSION)
    endif()
endforeach()