#include "device_pigpio.h"

std::atomic_bool Received_packet = 0;
uint8_t rxbuf[25] = {0x0};
std::mutex mutex_comm;

int main() {

    devicez test_i2c;
    std::array<float, MAX_MOTOR> encoders_pack_array;
    std::vector<int> encoders_pack;
    std::vector<int> vel_target_pack1 = {0,0,0,0};
    std::vector<int> vel_target_pack2 = {60,60,60,60};
    bool move_flag = 1;
    int count = 0;

    int motors_num = test_i2c.motors_detect();
    if ( motors_num == 0 ) {
        zos::log("NO motor detected\n");
    }else {
        zos::log("{} motor detected!\n", motors_num);
    }
    while (true) {
        if(move_flag) {
            test_i2c.set_motors_vel(vel_target_pack1);
            test_i2c.dribbler(0x00 | 0x0c);
        }else {
            test_i2c.set_motors_vel(vel_target_pack2);
            test_i2c.dribbler(0x01 | 0x0c);
        }
        encoders_pack_array = test_i2c.get_motors_vel();
        zos::log("encoder: {} {} {} {}\n", encoders_pack_array[0], encoders_pack_array[1], encoders_pack_array[2], encoders_pack_array[3]);

        // if(move_flag) {
        //     test_i2c.motors_write(vel_target_pack1);
        // }else {
        //     test_i2c.motors_write(vel_target_pack2);
        // }
        // encoders_pack = test_i2c.get_encoder();
        // zos::log("encoder: {} {} {} {}\n", encoders_pack[0], encoders_pack[1], encoders_pack[2], encoders_pack[3]);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(count++ > 200) {
            move_flag = !move_flag;
            count = 0;
        }
    }
}