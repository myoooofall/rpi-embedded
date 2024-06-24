#ifndef DEVICE_H
#define DEVICE_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>

#include "zos/utils/threadpool.h"
#include "zos/utils/rate.h"
#include "zos/log.h"
#include "config.h"

class device{
public:
    virtual void motors_device(int num, uint8_t *i2c_addr_t) = 0;
    virtual int motors_detect() = 0;
    virtual void motors_write(std::vector<int>& vel_pack) = 0;
    virtual void motors_write_single(int motor_id, int vel) = 0;
    virtual float shoot_chip(bool kick_mode, float kick_discharge_time) = 0;
    virtual void infrare_detect() = 0;
    virtual void dribbler(int dribble_val) = 0;

    virtual void buzzer_once(int freq) = 0;
    
    void output_test() {
        i2c_testmode = true;
    };

protected:
    int device_num = MAX_MOTOR;
    std::vector<int> vel_encoder = {0,0,0,0};
    int adc_val;

    int Rx_buf[MAX_MOTOR];
    bool i2c_testmode = false;
    int test_charge_count = 0;

private:
};

#endif