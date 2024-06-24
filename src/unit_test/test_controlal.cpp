#include "controlal.h"
using namespace std;
using namespace zos;

uint16_t Received_packet = 0;
uint32_t Total_Missed_Package_Num = 0;

uint8_t rxbuf[25] = {0x0};

std::string rxbuf_proto;
std::mutex mutex_comm;


int main(){
    
    controlal control(controlal::Mode::RealTime);
    Rate Rate_command(CAM_RATE);
    while(1){
        if(control.control_done())control.sendCommand(3000,2000,0,0);
        Rate_command.sleep();
    }
    return 0 ; 
}
