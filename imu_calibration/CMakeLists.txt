cmake_minimum_required(VERSION 3.16)

project(IMU_Communication)

set(LibMRAA /usr/local/lib/libmraa.so)

set(LIBS
    ${LIBS}
    ${LibMRAA}
    
)

# 添加可执行文件#
add_executable(imu_communication main.cpp)
add_executable(imu_calibration calibration.cpp)
# 链接MRAA库
target_link_libraries(imu_calibration ${LIBS})
target_link_libraries(imu_communication ${LIBS})