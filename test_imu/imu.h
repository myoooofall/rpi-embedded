#include <iostream>
#include "mraa/common.hpp"
#include "mraa/uart.hpp"

struct imu_data {
        float acc_x;
        float acc_y;
        float acc_z;
        float T_degree;
        float omega_x;  // degree/s
        float omega_y;
        float omega_z;
        float voltage;
        float theta_x;
        float theta_y;
        float theta_z;
        float version;
    } imu_status;

