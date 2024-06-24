#include "nrf2401.h"

uint16_t Received_packet = 0;
uint32_t Total_Missed_Package_Num = 0;
bool received_packet_flag = false;

uint8_t rxbuf[25] = {0x0};
uint8_t txbuf[25] = {0x0};
std::string rxbuf_proto;

std::mutex mutex_comm;

int main() {
    comm_2401 test;
    int count = 0;
    txbuf[0] = 0xff;
    txbuf[1] = 0x02;
    while(true) {
        if (test.get_receive_flag()) {
            uint8_t* rxbuf_ptr = test.get_rxbuf();
            std::copy(rxbuf_ptr, rxbuf_ptr+MAX_SIZE, rxbuf);
            test.set_receive_flag();
            zos::log("receive package: {} {} {} {} {}\n", rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4]);
            count++;
        }
        if (count > 50) {
            test.send(std::data(txbuf));
            count = 0;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return 0;
}
