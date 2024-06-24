#include <iostream>
#include "mraa/common.hpp"
#include "mraa/uart.hpp"
#include <vector>
#include <algorithm>
#include <iomanip>
#define UART_BUFF_SIZE 256

void printHex(const uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(data[i]) << " ";
    }
    std::cout << std::dec << std::endl;
}
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

int main() {
    mraa::Uart* uart=new mraa::Uart ("/dev/ttyAMA1");
    uart->setBaudRate(115200);
    uint8_t buff[UART_BUFF_SIZE]={0};
    std::string buff_str = uart->readStr(UART_BUFF_SIZE);
    std::copy(buff_str.begin(), buff_str.begin()+UART_BUFF_SIZE, buff);
    
    int length=buff_str.size();
    std::cout<<length<<std::endl;
    printHex(buff,length);

    std::vector<uint8_t> data(buff, buff + length);
    
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
    







    return 0;
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