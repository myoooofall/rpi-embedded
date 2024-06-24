#include "device_ROCKS.h"

devicez::devicez(int num, uint8_t *i2c_addr_t) : devices(num, {0}),shoot(PWM0_SHOOT),charge(GPIO_CHARGE),infrarein(GPIO_INFRARE_IN),infrareout(GPIO_INFRARE_OUT) {
    motors_device(num, i2c_addr_t);
    
    shoot.enable(true);
    charge.dir(mraa::DIR_OUT);
    infrarein.dir(mraa::DIR_OUT);
    infrareout.dir(mraa::DIR_IN);
}

void devicez::motors_device(int num, uint8_t *i2c_addr_t) {
    device_num = num;
    std::cout << num << " motors" << std::endl;
    for (int i=0; i<device_num; i++) {
        // if (i2c_addr_t)   i2c_addr[i] = i2c_addr_t[i];
        device.address(i2c_addr[i]);
        devices[i] = device;
    }
}

int devicez::motors_detect() {
    int motors_on = 0;
    for (int i=0; i<device_num; i++) {
        try {
            Rx_buf[i] = devices[i].readByte();
        }
        catch(const std::exception& e) {
            std::cerr << e.what() << '\n';
        }
        // Rx_buf[i] = wiringPiI2CRead(device[i]);
        if (Rx_buf[i] != -1)    motors_on++;
    }
    if (i2c_testmode && motors_on) {
        // Output
        std::cout << "huibao: ";
        for (int i=0; i<device_num; i++)    std::cout<< Rx_buf[i] << " ";
        std::cout << std::endl;
    }
    return motors_on;
}

void devicez::motors_write(std::vector<int>& vel_pack) {
    for (int i=0; i<device_num; i++) {
        devices[i].writeByte(vel_pack[i]);
        // wiringPiI2CWrite(device[i], vel_pack[i]);
        // delay(5);    // FIX
    }
    // Output
    if (i2c_testmode) {
        zos::log("motor write: {}\n", fmt::join(vel_pack, " "));
    }
}

void devicez::i2c_read(int* vel_encoder) {
    // int Curr_Vel = -1;
    for (int i=0; i<device_num; i++) {
        vel_encoder[i] = devices[i].readByte();
        // wiringPiI2CWrite(device[i], vel_pack[i]);
        // delay(5);    // FIX
    }
}

uint8_t devicez::shoot_chip(uint8_t Robot_Is_Boot_charged, uint8_t Robot_Boot_Power) {
    shoot.write(0);
    if(test_charge_count++ > 1000) {
        charge.write(HIGH);    // start charge
        std::cout << "Robot_Is_Boot_charged" << std::endl;
        test_charge_count = 0;
        Robot_Is_Boot_charged = 1;          // enable shoot
    }else if(test_charge_count == 20) {
        charge.write(LOW);    // stop charge
        shoot.write(0);
    }
    
    if(Robot_Boot_Power > 0 && Robot_Is_Boot_charged) {
        charge.write(LOW);    // stop charge
        shoot.write((Robot_Boot_Power<225)?(Robot_Boot_Power/300):0.75);  // duty ratio
        Robot_Is_Boot_charged = 0;
        test_charge_count = 0;
        std::cout << "shoot" << std::endl;
    }
    
    return Robot_Is_Boot_charged;
}

void devicez::infrare_detect() {}
void devicez::dribbler() {}