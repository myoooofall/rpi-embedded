#define IMU_DATA_LENGTH 256

#include <iostream>
#include "mraa/common.hpp"
#include "mraa/uart.hpp"
#include <vector>
#include <algorithm>
#include <iomanip>

    // 缓冲区用于存储读取的数据
    //uint8_t*buffer;
uint8_t buffer[IMU_DATA_LENGTH] = {0};
int read_imu_raw(mraa::Uart* uart);
int read_imu(mraa::Uart* uart);
bool sumcrc(uint8_t* data) ;
struct imu_data {
        float acc_x;
        float acc_y;
        float acc_z;
        float T_degree;
        float omega_x;  // degree/s
        float omega_y;
        float omega_z;
        float voltage;
        float theta_x;
        float theta_y;
        float theta_z;
        float version;
    } imu_status;
void printHex(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}
int main() {
    // 初始化串口
    mraa::Uart *uart1=new mraa::Uart ("/dev/ttyAMA1");
    uart1->setBaudRate(115200);
    uart1->setFlowcontrol(false, true);
    uart1->setMode(8,mraa::UART_PARITY_NONE , 0);
    uart1->setTimeout(10,10,10);

    mraa::Uart *uart=new mraa::Uart ("/dev/ttyAMA0");
    uart->setBaudRate(921600);
    uart->setFlowcontrol(false, true);
    uart->setMode(8,mraa::UART_PARITY_NONE , 0);
    uart->setTimeout(10,10,10);
    // 缓冲区用于存储读取的数据
    

    while (true) {
        // 从串口读取数据
       
       read_imu(uart);
       
    //    std::cout<<imu_status.acc_x<<std::endl;
    //    std::cout<<imu_status.acc_y<<std::endl;
    //    std::cout<<imu_status.acc_z<<std::endl;
    //    std::cout<<imu_status.omega_x<<std::endl;
    //    std::cout<<imu_status.omega_y<<std::endl;
    //    std::cout<<imu_status.omega_z<<std::endl;
    //    std::cout<<imu_status.theta_x<<std::endl;
    //    std::cout<<imu_status.theta_y<<std::endl;
    //    std::cout<<imu_status.theta_z<<std::endl;
    //    std::cout<<imu_status.T_degree<<std::endl;
    //    std::cout<<imu_status.voltage<<std::endl;
    //     std::cout<<imu_status.version<<std::endl;

        std::cout<<"-------------------------------------"<<std::endl;
        read_imu_raw(uart1);
    }

    return 0;
}


int read_imu(mraa::Uart* uart) {
    int length = read_imu_raw(uart);
    std::vector<uint8_t> data(buffer, buffer + length);
    
    std::vector<uint8_t> pattern_acc = {0x55, 0x51};
    std::vector<uint8_t> pattern_omega = {0x55, 0x52};
    std::vector<uint8_t> pattern_theta = {0x55, 0x53};
    {
        std::vector<uint8_t>::iterator it = std::search(data.begin(), data.end(), pattern_acc.begin(), pattern_acc.end());
        if (it != data.end()) {
            size_t index = std::distance(data.begin(), it);
            // ESP_LOGI("imu", "imu acc found at index %d", index);
            // SUMCRC=0x55+TYPE+DATA1L+DATA1H+DATA2L+DATA2H+DATA3L+DATA3H+DATA4L+DATA4H
            uint8_t crc_data[11];
            std::copy(it, it+11, crc_data);
            if(!sumcrc(crc_data)) {
                 std::cout<<"imu theta crc error"<<std::endl;
            } else {
                // TODO: calculate acc
                imu_status.acc_x = (short)(((short)data[index + 3] << 8) | data[index + 2]) / 32768.0 * 16 * 9.8;
                imu_status.acc_y = (short)(((short)data[index + 5] << 8) | data[index + 4]) / 32768.0 * 16 * 9.8;
                imu_status.acc_z = (short)(((short)data[index + 7] << 8) | data[index + 6]) / 32768.0 * 16 * 9.8;
                imu_status.T_degree = (short)(((short)data[index + 9] << 8) | data[index + 8]) / 32768.0 * 96.38 + 36.53;
                // ESP_LOGE("imu", "imu acc_x_raw: %d, %d, %d, %d", data[index+0], data[index + 1], data[index + 2], data[index + 3]);
                // ESP_LOGI("imu", "imu acc: %f, %f, %f, %f", imu_status.acc_x, imu_status.acc_y, imu_status.acc_z, imu_status.T_degree);
            }
        } else {
            std::cout<<"imu acc not found"<<std::endl;
        }
    }
    {
        std::vector<uint8_t>::iterator it = std::search(data.begin(), data.end(), pattern_omega.begin(), pattern_omega.end());
        if (it != data.end()) {
            size_t index = std::distance(data.begin(), it);
            uint8_t crc_data[11];
            std::copy(it, it+11, crc_data);
            if(!sumcrc(crc_data)) {
                std::cout<<"imu theta crc error"<<std::endl;
            } else {
                imu_status.omega_x = (short)(((short)data[index + 3] << 8) | data[index + 2]) / 32768.0 * 2000;
                imu_status.omega_y = (short)(((short)data[index + 5] << 8) | data[index + 4]) / 32768.0 * 2000;
                imu_status.omega_z = (short)(((short)data[index + 7] << 8) | data[index + 6]) / 32768.0 * 2000;
                imu_status.voltage = (short)(((short)data[index + 9] << 8) | data[index + 8]) / 100.0;
            }
        } else {
           std::cout<<"imu acc not found"<<std::endl;
        }
    }
    {
        std::vector<uint8_t>::iterator it = std::search(data.begin(), data.end(), pattern_theta.begin(), pattern_theta.end());
        if (it != data.end()) {
            size_t index = std::distance(data.begin(), it);
            uint8_t crc_data[11];
            std::copy(it, it+11, crc_data);
            if(!sumcrc(crc_data)) {
                std::cout<<"imu theta crc error"<<std::endl;
            } else {
                imu_status.theta_x = (short)(((short)data[index + 3] << 8) | data[index + 2]) / 32768.0 * 180;
                imu_status.theta_y = (short)(((short)data[index + 5] << 8) | data[index + 4]) / 32768.0 * 180;
                imu_status.theta_z = (short)(((short)data[index + 7] << 8) | data[index + 6]) / 32768.0 * 180;
                imu_status.version = (short)(((short)data[index + 9] << 8) | data[index + 8]);
            }
        } else {
            std::cout<<"imu acc not found"<<std::endl;
        }
    }
    return length;
}

int read_imu_raw(mraa::Uart* uart) {
    
    
    

    std::string buff_str = uart->readStr(IMU_DATA_LENGTH);
    int length=buff_str.size();


    if(length > IMU_DATA_LENGTH) length= IMU_DATA_LENGTH;

    if(length==IMU_DATA_LENGTH){
        std::cout<<"get status num: "<<IMU_DATA_LENGTH <<std::endl;
    }
    else if(length<IMU_DATA_LENGTH){
        std::cout<<"get status num: "<<length<<std::endl;
    }
    std::copy(buff_str.begin(), buff_str.begin()+IMU_DATA_LENGTH, buffer);
    
    printHex(buffer, length);
    return length;
}

bool sumcrc(uint8_t* data) {
            uint16_t sum = 0x0;
            for (size_t i = 0; i < 10; i++) {
                sum += data[i];  // 累加数据
            }
            uint8_t crc = sum & 0xFF;  // 取校验和的低8位
            // ESP_LOGD("imu crc", "sumcrc: %d, real: %d", crc, data[10]);
            return (crc == data[10]);
        }
