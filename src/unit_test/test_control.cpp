#include <iostream>
#include "robotz.h"

std::atomic_bool Received_packet = 0;

uint8_t rxbuf[25] = {0x0};

std::mutex mutex_comm;

int main(){
    zos::status("starting...\n");
    robotz zjunlict(robotz::COMM_TYPE_WIFI);
    while (true){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(zjunlict.ms_since_last_cmd_pack() > 100){
            zjunlict.stand();
        }
    }
    return 0;
}