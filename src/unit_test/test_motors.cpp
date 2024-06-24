#include "robotz.h"
// #include "wifiz.h"

uint16_t Received_packet = 0;
uint32_t Total_Missed_Package_Num = 0;

uint8_t rxbuf[25] = {0x0};
std::string rxbuf_proto;

std::mutex mutex_comm;

int main() {
    // Robot Init
    robotz zjunlict;
    // zjunlict.testmode_on();
    int missed_pack = 0;

    while (1)
    {   
        // Detect receive pack
        if (Received_packet) {
            zjunlict.get_new_pack();
            missed_pack = 0;
        }else if(missed_pack++ > 500) {
            zjunlict.stand();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
