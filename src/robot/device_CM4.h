#ifndef I2C_H
#define I2C_H

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "device.h"

#define GPIO_INFRARE_IN     4
#define GPIO_INFRARE_OUT    5
#define PWM0_SHOOT          26
#define GPIO_CHARGE         0

class devicez : public device{
public:
    devicez(int num = MAX_MOTOR, uint8_t *i2c_addr_t = NULL);
    void motors_device(int num, uint8_t *i2c_addr_t) override;
    int motors_detect() override;
    void motors_write(std::vector<int>& vel_pack) override;
    void motors_write_single(int motor_id, int vel) override;
    uint8_t shoot_chip(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) override;
    void infrare_detect() override;
    void dribbler() override;

    uint8_t shoot_test(uint8_t Robot_Boot_Power);
    void adc_test();
    float adc_cap_vol();

private:
    std::vector<std::jthread> i2c_th_single;
    int motors_i2c[MAX_MOTOR];
    int adc_i2c;
};

#endif