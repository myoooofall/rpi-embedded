#ifndef CONFIG_H
#define CONFIG_H

#include <chrono>

#define PI  3.14159
#define MAX_MOTOR   4
#define TX_BUF_SIZE 25
#define CAM_RATE 75
#define CONTROL_DEBUGGER 1

#define DEBUG_MODE

namespace config {
    
    constexpr uint8_t robot_id = 0x06;
    constexpr uint8_t motors_addr[4] = {0x20,0x22,0x24,0x26};
    constexpr uint8_t dribbler_addr = 0x28;
    // constexpr uint8_t motors_addr[4] = {0x28,0x29,0x30,0x31};
    constexpr int i2c_bus = 1;
    constexpr uint8_t adc_addr = 0x48;
    constexpr float adc_cap_vol_k = 2.23; // 5/255*1008.87/8.87
    // nrf2401
    constexpr int radio_tx_ce_pin = 27;
    constexpr int radio_tx_csn = 0; // <a>*10+<b>; spidev1.0 is 10, spidev1.1 is 11 etc..
    constexpr int radio_rx_ce_pin = 22;
    constexpr int radio_rx_csn = 0;

    // wifi
    constexpr int receive_port = 14234;
    constexpr int send_single_port = 14134;
    constexpr int send_multicast_port = 13134;
    constexpr char multicast_addr[] = "225.225.225.225";
    const std::string multicast_if_prefix = "192.168.31";
    constexpr int multicast_freq = 10;// Hz
    constexpr int sendback_freq = 250;// Hz

    constexpr int udp_freq = 1000;
    constexpr int control_cmd_freq = 300;
    constexpr int robot_freq = 250;
    
    // wheel
    constexpr double car_angle_front = 58.0*PI/180;
    constexpr double car_angle_back = 45.0*PI/180;
    constexpr double vel_ratio = 3.18/2.8*10;
    // constexpr double vel_ratio = 0.520573;

    // control params;
    constexpr double dt_us = 1000000*1/control_cmd_freq; //s
    constexpr double a_max = 8.0; // m/s^2
    constexpr double v_max = 6.0; // m/s
}

#endif