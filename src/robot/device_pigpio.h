#ifndef I2C_H
#define I2C_H

#include "device.h"
#include <pigpio.h>
#include "mraa/common.hpp"
#include "mraa/uart.hpp"

#define SHOOT_MODE  0
#define CHIP_MODE   1
// using BCM number
#define GPIO_SHOOT          23
#define GPIO_CHIP           24
#define GPIO_BUZZER         12

#define GPIO_TX_EN          17
#define GPIO_RX_EN          18

#define GPIO_LED0           19
#define GPIO_LED1           20
#define GPIO_LED2           21

#define UART_BUFF_SIZE 3
#define IMU_DATA_LENGTH 256
class devicez : public device{
public:
    devicez(int num = MAX_MOTOR, uint8_t *i2c_addr_t = NULL);
    void motors_device(int num, uint8_t *i2c_addr_t) override;
    int motors_detect() override;
    void motors_write(std::vector<int>& vel_pack) override;
    void motors_write_single(int motor_id, int vel) override;
    float shoot_chip(bool kick_mode, float kick_discharge_time) override;
    void infrare_detect() override;
    void dribbler(int dribble_val) override;

    void buzzer_once(int freq) override;
    void buzzer_start();
    void buzzer_set_freq_num();

    void led_flash(int led_id, int level);
    // void led_flash() {led_flash(0,1);};
    // TODO: led toggle
    uint8_t buffer[IMU_DATA_LENGTH] = {0};

    std::vector<int> read_nano_uart();
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
    mraa::Uart* uart1;
   

    int read_imu_raw(mraa::Uart* uart);
    int read_imu(mraa::Uart* uart);
    bool sumcrc(uint8_t* data) ;

    // std::vector<int> get_encoder();
    // std::array<float, MAX_MOTOR> get_encoder_array();
    std::array<float, MAX_MOTOR> get_motors_vel();
    void set_motors_vel(std::vector<int>& vel_pack);

    std::vector<int> get_motors_pid();
    void set_motors_pid(std::vector<int>& pid_pack);
    // void motors_pid_write_single(int motor_id, char* pid_pack);

    // TODO: i2c-read-write any size devices/buffer


    void inline static set_nrf2401_en(int mode=0) {
        gpioSetMode(GPIO_TX_EN, PI_OUTPUT);
        gpioSetMode(GPIO_RX_EN, PI_OUTPUT);
        if(mode == 0) {
            gpioWrite(GPIO_TX_EN, PI_HIGH);
            gpioWrite(GPIO_RX_EN, PI_LOW);
        }else if(mode == 1) {
            gpioWrite(GPIO_TX_EN, PI_LOW);
            gpioWrite(GPIO_RX_EN, PI_HIGH);
        }
    }; // mode 0: default  1: reverse

    ~devicez() {
        gpioTerminate();
    }

private:
    std::jthread _jthread4i2c;
    std::mutex mutex_i2c;
    std::mutex mutex_uart;
    std::vector<std::jthread> i2c_th_single;
    int motors_i2c_handle[MAX_MOTOR];
    int dribbler_i2c_handle;
    int adc_i2c_handle;

    mraa::Uart* uart;
    void write_uart(uint8_t* buff);
    void read_uart(uint8_t* buff);

    uint8_t motors_i2c_addr[4];
    // motors thread
    void _motors_run_thread(std::stop_token _stop_token);
    std::array<std::atomic_int, MAX_MOTOR> vel_target_atomic;  // 0.1 rad/s
    std::array<std::atomic_int, MAX_MOTOR> vel_encoders_atomic;  // 0.1 rad/s
};

#endif