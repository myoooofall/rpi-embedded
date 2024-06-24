#include "robotz.h"
// #include "wifiz.h"

std::atomic_bool Received_packet = 0;

uint8_t rxbuf[25] = {0x0};
std::string rxbuf_proto;

std::mutex mutex_comm;

int main() {
    uint32_t Total_Missed_Package_Num = 0;
    bool received_packet_flag;

    // Robot Init
    robotz zjunlict;
    // zjunlict.testmode_on();
    // TODO: wait for robot udp init
    while ( !Received_packet )
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    while (true)
    {
        // Detect receive pack
        received_packet_flag = false;
        if (Received_packet) {  // Callback of udp receive
            received_packet_flag = zjunlict.get_new_pack();
            if (received_packet_flag == false) {
                if (Total_Missed_Package_Num++ >= 500) {    // Missing package for 1 seconds
                    zjunlict.stand();   // Set to 0
                    Total_Missed_Package_Num = 0;
                    zos::warning("package missed\n");
                }
            }else {
                zos::log("new pack\n");
            }
        }
        
        // period time test
        // zjunlict.period_test();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
