#include "device_CM4.h"

devicez::devicez(int num, uint8_t *i2c_addr_t) : i2c_th_single(num) {
    motors_device(num, i2c_addr_t);
    adc_i2c = wiringPiI2CSetup(config::adc_addr);
    wiringPiI2CWrite(adc_i2c, 0x40);

    // shoot
    pinMode(GPIO_CHARGE, OUTPUT);
    pinMode(PWM0_SHOOT, PWM_OUTPUT);
    pwmSetClock(320);
    // infrare
    // pinMode(GPIO_INFRARE_OUT, INPUT);
    // pinMode(GPIO_INFRARE_IN, OUTPUT);
}

void devicez::motors_device(int num, uint8_t *i2c_addr_t) {
    device_num = num;
    if (wiringPiSetup() != 0) {
        std::cout << "wiringPi error!" << std::endl;
    }

    if(i2c_addr_t)   std::copy(i2c_addr_t, i2c_addr_t+device_num, motors_i2c);
    else {
        std::copy(config::motors_addr, config::motors_addr+device_num, motors_i2c);
        zos::info("use default i2c address\n");
    }
    
    for (int i=0; i<device_num; i++) {
        motors_i2c[i] = wiringPiI2CSetup(motors_i2c[i]);
        // std::cout << i << ": " << std::hex << i2c_addr[i] << " ";
    }
}

int devicez::motors_detect() {
    int motors_on = 0;
    for (int i=0; i<device_num; i++) {
        vel_encoder[i] = wiringPiI2CRead(motors_i2c[i]); // TODO: read specific bits
        if (vel_encoder[i] != -1)    motors_on++;
    }
    if (i2c_testmode && motors_on) {
        zos::status("motor encoder: {}\n", fmt::join(vel_encoder, " "));
    }
    return motors_on;
}

void devicez::motors_write(std::vector<int>& vel_pack) {
    for (int i=0; i<device_num; i++) {
        i2c_th_single[i] = std::jthread(&devicez::motors_write_single, this, i, abs(vel_pack[i]));
        // i2c_th_single[i].join();
    }
    // Output
    if (i2c_testmode) {
        zos::log("motor write: {}\n", fmt::join(vel_pack, " "));
        zos::status("motor encoder: {}\n", fmt::join(vel_encoder, " "));
    }
}

void devicez::motors_write_single(int motor_id, int vel) {
    wiringPiI2CWrite(motors_i2c[motor_id], abs(vel));
    // zos::log("motor id: {}, vel: {}\n", motor_id, vel);
    vel_encoder[motor_id] = wiringPiI2CRead(motors_i2c[motor_id]); // TODO: read specific bits
}

uint8_t devicez::shoot_chip(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) {
    pwmWrite(PWM0_SHOOT, 0);

    if(test_charge_count++ > 1000) {
        digitalWrite(GPIO_CHARGE, HIGH);    // start charge
        zos::info("Robot is boot charged\n");
        test_charge_count = 0;
        Robot_Is_Boot_charged = 1;          // enable shoot
    }else if(test_charge_count == 20) {
        digitalWrite(GPIO_CHARGE, LOW);    // stop charge
        pwmWrite(PWM0_SHOOT, 0);
    }
    
    if(Robot_Boot_Power > 0 && Robot_Is_Boot_charged) {
        digitalWrite(GPIO_CHARGE, LOW);    // stop charge
        pwmWrite(PWM0_SHOOT, Robot_Boot_Power*3);
        Robot_Is_Boot_charged = 0;
        test_charge_count = 0;
        zos::info("shoot");
    }
    return Robot_Is_Boot_charged;
}

uint8_t devicez::shoot_test(uint8_t Robot_Boot_Power) {
    if(Robot_Boot_Power > 0) {
        pwmWrite(PWM0_SHOOT, Robot_Boot_Power);
        test_charge_count = 0;
        zos::info("shoot");
    }
    pwmWrite(PWM0_SHOOT, 0);
    return 0;
}

void devicez::adc_test() {
    wiringPiI2CWrite(adc_i2c, 0x41);

    adc_val = wiringPiI2CRead(adc_i2c);
    zos::status("adc value: {}\n", adc_val);
}

float devicez::adc_cap_vol() {
    wiringPiI2CWrite(adc_i2c, 0x40);
    adc_val = wiringPiI2CRead(adc_i2c);
    float cap_vol = adc_val * config::adc_cap_vol_k;
    zos::status("cap voltage: {}\n", cap_vol);
    return cap_vol;
}

void devicez::infrare_detect() {}
void devicez::dribbler() {}