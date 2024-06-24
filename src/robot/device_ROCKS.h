#ifndef I2C_H
#define I2C_H

#include <mraa/common.hpp>
#include <mraa/gpio.hpp>
#include <mraa/i2c.hpp>
#include <mraa/pwm.hpp>

#include "device.h"

#define MAX_MOTOR 4

#define LOW     0
#define HIGH    1

#define GPIO_INFRARE_IN     16  // GPIO2_B2
#define GPIO_INFRARE_OUT    18  // GPIO2_B1
#define PWM0_SHOOT          13  // FIXME: use pin PWM_R
#define GPIO_CHARGE         22

class devicez : public device{
public:
    devicez(int num = MAX_MOTOR, uint8_t *i2c_addr_t = NULL);

    void motors_device(int num, uint8_t *i2c_addr_t) override;
    int motors_detect() override;
    void motors_write(std::vector<int>& vel_pack) override;
    uint8_t shoot_chip(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) override;
    void infrare_detect() override;
    void dribbler() override;
    
    void i2c_read(int* vel_encoder);

private:
    mraa::I2c device = mraa::I2c(0);
    std::vector<mraa::I2c> devices;

    mraa::Pwm shoot;
    mraa::Gpio charge;
    mraa::Gpio infrarein;
    mraa::Gpio infrareout;
};

#endif