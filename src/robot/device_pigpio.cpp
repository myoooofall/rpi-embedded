#include "device_pigpio.h"

devicez::devicez(int num, uint8_t *i2c_addr_t) : i2c_th_single(num) {
    // int cfg = gpioCfgGetInternals();
    // cfg |= PI_CFG_NOSIGHANDLER;  // (1<<10)
    // gpioCfgSetInternals(cfg);
    auto _code_ = gpioInitialise();
    if (_code_ < 0) {
        zos::error("pigpio error!{}\n",_code_);
    }else {
        zos::log("pigpio init\n");
    }
    
    motors_device(num, i2c_addr_t);
    dribbler_i2c_handle = i2cOpen(config::i2c_bus, config::dribbler_addr, 0);

    _jthread4i2c = std::jthread(std::bind(&devicez::_motors_run_thread,this,std::placeholders::_1));

    // shoot
    gpioSetMode(GPIO_SHOOT, PI_OUTPUT);
    gpioSetMode(GPIO_CHIP, PI_OUTPUT);
    gpioWrite(GPIO_SHOOT, PI_LOW);
    gpioWrite(GPIO_CHIP, PI_LOW);
    // pwmSetClock(320);
    // infrare
    // pinMode(GPIO_INFRARE_OUT, INPUT);
    // pinMode(GPIO_INFRARE_IN, OUTPUT);
    // buzzer
    gpioSetMode(GPIO_BUZZER, PI_OUTPUT);
    buzzer_start();



    uart1=new mraa::Uart ("/dev/ttyAMA0");
    uart1->setBaudRate(921600);
    uart1->setFlowcontrol(false, true);
    uart1->setMode(8,mraa::UART_PARITY_NONE , 0);
    uart1->setTimeout(10,10,10);

  
    try {
        uart = new mraa::Uart("/dev/ttyAMA1");
    } catch (std::exception& e) {
        std::cerr << "Error while setting up raw UART, do you have a uart?" << std::endl;
        std::terminate();
    }
    if (uart->setBaudRate(115200) != mraa::SUCCESS) {
        std::cerr << "Error setting parity on UART" << std::endl;
    }
}

void devicez::buzzer_start() {
    std::thread _buzzer = std::thread([this] {
        buzzer_once(500);
        buzzer_once(1600);
    });
    _buzzer.join();
}
void devicez::buzzer_set_freq_num() {
    std::thread _buzzer = std::thread([this] {
        buzzer_once(1600);
    });
    _buzzer.join();
}
void devicez::buzzer_once(int freq) {
    gpioSetPWMfrequency(GPIO_BUZZER, freq);
    gpioPWM(GPIO_BUZZER, 20);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    gpioPWM(GPIO_BUZZER, 0);
}

void led_flash(int led_id, int level) {
    int gpio_led;
    switch (led_id) {
        case 0:
            gpio_led = GPIO_LED0;
            break;
        case 1:
            gpio_led = GPIO_LED1;
            break;
        case 2:
            gpio_led = GPIO_LED2;
            break;
    }
    gpioSetMode(gpio_led, PI_OUTPUT);
    gpioWrite(gpio_led, level);
}

void devicez::motors_device(int num, uint8_t *i2c_addr_t) {
    device_num = num;

    if(i2c_addr_t) {
        std::copy(i2c_addr_t, i2c_addr_t+device_num, motors_i2c_addr);
        zos::info("use manual i2c address\n");
    }else {
        std::copy(config::motors_addr, config::motors_addr+device_num, motors_i2c_addr);
        zos::info("use default i2c address\n");
    }
    
    for (int i=0; i<device_num; i++) {
        motors_i2c_handle[i] = i2cOpen(config::i2c_bus, motors_i2c_addr[i], 0);
        // std::cout << i << ": " << std::hex << i2c_addr[i] << " ";
    }
}

int devicez::motors_detect() {
    int motors_on = 0;
    for (int i=0; i<device_num; i++) {
        // FIXME: i2c read
        vel_encoder[i] = i2cReadByte(motors_i2c_handle[i]); // TODO: read specific bits
        if (vel_encoder[i] != PI_I2C_READ_FAILED)    motors_on++;
    }
    // FIXME: i2c read
    // device_num = motors_on;
    if (device_num && i2c_testmode) {
        zos::status("motor encoder: {}\n", fmt::join(vel_encoder, " "));
    }
    return motors_on;
}

void devicez::dribbler(int dribble_val) {
    // int dribble_val = (0x0c) | (0x03);
    std::scoped_lock lock(mutex_i2c);
    // zos::log("dribble val: {:#04x}\n", dribble_val);
    i2cWriteByte(dribbler_i2c_handle, dribble_val);
    // i2cWriteDevice(dribbler_i2c_handle, (char*)dribble_val, 1);
}

void devicez::_motors_run_thread(std::stop_token _stop_token) {
    int count_i2c = 0;
    bool test_output = false;
    while (true) {
        if (count_i2c++ > 1000) {
            test_output = true;
            zos::warning("i2c rx/tx for 1000 times\n");
            count_i2c = 0;
        }else{
            test_output = false;
        }
        for (int motor_id=0; motor_id<device_num; motor_id++) {
            // write
            uint8_t vel_pack_temp[3] = {0x0};
            uint8_t encoder_pack[3] = {0x0};
            vel_pack_temp[0] = 0xfa;
            vel_pack_temp[1] = ((abs(vel_target_atomic[motor_id]) >> 8) & 0x1f) | (((vel_target_atomic[motor_id]>=0)?0:1) << 5) | (0x01 << 6);
            vel_pack_temp[2] = (abs(vel_target_atomic[motor_id]) & 0xff);
            {
                std::scoped_lock lock(mutex_i2c);
                // for(int i=0; i<3; i++) {
                //     i2cWriteByte(motors_i2c_handle[motor_id], vel_pack_temp[i]);
                // }
                i2cWriteDevice(motors_i2c_handle[motor_id], (char*)vel_pack_temp, 3);
                // zos::log("motor id: {}, vel_int: {}, vel_pack_temp: {:#04x} {:#04x} {:#04x}\n", motor_id, vel, (char)vel_pack_temp[0], (char)vel_pack_temp[1], (char)vel_pack_temp[2]);
                // std::this_thread::sleep_for(std::chrono::milliseconds(5));
                // std::this_thread::sleep_for(std::chrono::microseconds(1));
                // for(int i=0; i<3; i++) {
                //     encoder_pack[i] = i2cReadByte(motors_i2c_handle[motor_id]);
                // }

                i2cReadDevice(motors_i2c_handle[motor_id], (char*)encoder_pack, 3);
                // zos::warning("encoder pack: {} {} {}\n", encoder_pack[0], encoder_pack[1], encoder_pack[2]);
            }
            // encoder
            if (encoder_pack[0] == 0xfa) {
                float vel_temp = ((encoder_pack[1] & 0x1f) << 8) + encoder_pack[2];
                vel_encoders_atomic[motor_id] = ((encoder_pack[1] & 0x20)>>5)?(-vel_temp):vel_temp;
                // zos::status("encoder id: {}, vel_encoders_atomic: {}\n", motor_id, vel_encoders_atomic[motor_id]);
            }else {
                vel_encoders_atomic[motor_id] = 0;
                // zos::warning("wrong pack: {} {} {}\n", encoder_pack[0], encoder_pack[1], encoder_pack[2]);
            }
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    }
}

void devicez::set_motors_vel(std::vector<int>& vel_pack) {
    std::copy_n(vel_pack.begin(), MAX_MOTOR, vel_target_atomic.begin());
}

std::array<float, MAX_MOTOR> devicez::get_motors_vel() {
    std::array<float, MAX_MOTOR> vel_encoders_array;
    std::copy_n(vel_encoders_atomic.begin(), MAX_MOTOR, vel_encoders_array.begin());
    return vel_encoders_array;
}

void devicez::motors_write(std::vector<int>& vel_pack) {
    for (int i=0; i<device_num; i++) {
        i2c_th_single[i] = std::jthread(&devicez::motors_write_single, this, i, vel_pack[i]);
        // i2c_th_single[i].join();
    }
    // Output
    // if (device_num && i2c_testmode) {
    //     zos::log("motor write: {}\n", fmt::join(vel_pack, " "));
    //     zos::status("motor encoder: {}\n", fmt::join(vel_encoder, " "));
    // }
}

void devicez::motors_write_single(int motor_id, int vel) {
    uint8_t vel_pack[3] = {0x0};
    uint8_t encoder_pack[3] = {0x0};
    vel_pack[0] = 0xfa;
    vel_pack[1] = ((abs(vel) >> 8) & 0x1f) | (((vel>=0)?0:1) << 5) | (0x01 << 6);
    vel_pack[2] = (abs(vel) & 0xff);
    {
        std::scoped_lock lock(mutex_i2c);
        i2cWriteDevice(motors_i2c_handle[motor_id], (char*)vel_pack, 3);
        // zos::log("motor id: {}, vel_int: {}, vel_pack: {:#04x} {:#04x} {:#04x}\n", motor_id, vel, (char)vel_pack[0], (char)vel_pack[1], (char)vel_pack[2]);
        int read_status = i2cReadDevice(motors_i2c_handle[motor_id], (char*)encoder_pack, 3);
        // for(int i=0; i<3;i++) {
        //     encoder_pack[i] = i2cReadByte(motors_i2c_handle[i]);
        // }
        if(read_status > 0) {
            zos::warning("#{} encoder pack: {} {} {}\n", motor_id, encoder_pack[0], encoder_pack[1], encoder_pack[2]);
        }
    }
    
    // unit rad/s * 10
    if (encoder_pack[0] == 0xfa) {
        int vel_temp = ((encoder_pack[1] & 0x1f) << 8) + encoder_pack[2];
        vel_encoder[motor_id] = ((encoder_pack[1] & 0x20)>>5)?(-vel_temp):vel_temp;
        // zos::status("encoder id: {}, vel_encoder: {}\n", motor_id, vel_encoder[motor_id]);
    }else {
        vel_encoder[motor_id] = 0;
        // zos::warning("wrong pack: {} {} {}\n", encoder_pack[0], encoder_pack[1], encoder_pack[2]);
    }
    // i2cWriteByte(motors_i2c_handle[motor_id], abs(vel));
    // zos::log("motor id: {}, vel_int: {}\n", motor_id, vel);
    // encoder_read_single(motor_id);
    // vel_encoder[motor_id] = i2cReadByte(motors_i2c_handle[motor_id]); // TODO: read specific bits
    // zos::status("motor id: {}, vel_encoder: {}\n", motor_id, vel_encoder[motor_id]);
}

float devicez::shoot_chip(bool kick_mode, float kick_discharge_time) {
    int kick_flag = -1;
    int kick_gpio;
    if(kick_mode == SHOOT_MODE) {
        kick_gpio = GPIO_SHOOT;
    }else if(kick_mode == CHIP_MODE){
        kick_gpio = GPIO_CHIP;
    }else {
        zos::error("wrong kick mode\n");
        return -1;
    }

    if(kick_discharge_time > 0) {
        // std::jthread th_shoot([this, kick_gpio, kick_discharge_time] {
            auto time_pre = std::chrono::steady_clock::now();
            std::chrono::duration<int, std::nano> _step = std::chrono::microseconds(1);
            gpioWrite(kick_gpio, PI_HIGH);
            // zos::log("shoot: {}     ", _step*kick_discharge_time);
            std::this_thread::sleep_for(_step*(int)kick_discharge_time);
            gpioWrite(kick_gpio, PI_LOW);
            auto time_now = std::chrono::steady_clock::now();
            auto step = (time_now - time_pre);
            zos::status("kick power: {}, period time: {}, vol remain: {}\n", (int)kick_discharge_time, step.count()/1000, read_nano_uart()[1]);
        // });
        // th_shoot.detach();
        kick_flag = kick_mode;
    }else {
        zos::log("kick power: {}, low voltage: {}, boot power: {}\n", (int)kick_discharge_time, read_nano_uart()[1], (int)kick_discharge_time);
    }
    // gpioWrite(kick_gpio, PI_LOW);
    return kick_flag;
}

std::vector<int> devicez::get_motors_pid() {
    
    // return vel_encoder;
    char pid_req_pack_temp[3] = {0xf9};
    char pid_get_pack_temp[3] = {0x0};
    std::vector<int> pid_real_temp(8, 0x0);
    for (int motor_id=0; motor_id<device_num; motor_id++) {
        
        int try_count = 0;
        while (pid_get_pack_temp[0] != 0x95 && try_count++ < 5) {
            std::scoped_lock lock(mutex_i2c);
            i2cWriteDevice(motors_i2c_handle[motor_id], (char*)pid_req_pack_temp, 3);
            i2cReadDevice(motors_i2c_handle[motor_id], (char*)pid_get_pack_temp, 3);
        }
        if(pid_get_pack_temp[0] == 0x95) {
            pid_real_temp[motor_id*2] = pid_get_pack_temp[1];
            pid_real_temp[motor_id*2+1] = pid_get_pack_temp[2];
            

        }else {
            zos::warning("motor {} get pid failed\n", motor_id);
            pid_real_temp[motor_id*2] = 0;
            pid_real_temp[motor_id*2+1] = 0;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    return pid_real_temp;
}

void devicez::set_motors_pid(std::vector<int>& pid_pack) {
    char pid_pack_temp_long[8];
    std::copy(begin(pid_pack), end(pid_pack), pid_pack_temp_long);
    char pid_pack_temp[3] = {0xf5};
    // zos::log("pid: {:#04x} {:#04x} {:#04x}\n", pid_pack_temp[0], pid_pack_temp[1], pid_pack_temp[2]);
    // FIXME: i<device_num

    for (int motor_id=0; motor_id<device_num; motor_id++) {
        
        int try_count = 0;
        pid_pack_temp[0] = 0xf5;
        pid_pack_temp[1] = pid_pack_temp_long[motor_id*2];
        pid_pack_temp[2] = pid_pack_temp_long[motor_id*2+1];
        
        if ((pid_pack_temp[1]+pid_pack_temp[2]) == 0) {
            zos::log("motor {} pid is empty\n", motor_id);
            continue;
        }
        uint8_t pid_done_flag = 0x00;
        while (pid_done_flag != 0x59 && try_count++ < 10) {
            std::scoped_lock lock(mutex_i2c);
            i2cWriteDevice(motors_i2c_handle[motor_id], pid_pack_temp, 3);

            // char pid_report_temp[2] = {0x0};
            // i2cReadDevice(motors_i2c_handle[motor_id], (char*)pid_report_temp, 2);
            // zos::log("pid setting: {:#04x} {:#04x}\n", pid_report_temp[0], pid_report_temp[1]);

            pid_done_flag = i2cReadByte(motors_i2c_handle[motor_id]);
            // zos::log("motor {} pid setting {:#04x}\n", motor_id, pid_done_flag);
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        
        if(try_count >= 10){
            zos::error("motor {} set pid failed\n", motor_id);
        }else {
            // zos::log("motor {} pid set already\n", motor_id);
        }
    }
}

// void devicez::motors_pid_write_single(int motor_id, char* pid_pack) {
//     std::scoped_lock lock(mutex_i2c);
//     do {
//         i2cWriteDevice(motors_i2c_handle[motor_id], pid_pack, 3);
//         std::this_thread::sleep_for(std::chrono::microseconds(100));
//     }while (i2cReadByte(motors_i2c_handle[motor_id]) != 0xcc);
//     zos::log("motor {} pid set already\n");
// }

void devicez::infrare_detect() {}

void devicez::write_uart(uint8_t* buff) {
    std::string buff_str(buff, buff+UART_BUFF_SIZE);
    std::scoped_lock lock(mutex_uart);
    uart->writeStr(buff_str);
    // std::cout << "receive pack: " << buff_str << std::endl;
}

void devicez::read_uart(uint8_t* buff) {
    // std::scoped_lock lock(mutex_uart);
    std::string buff_str = uart->readStr(UART_BUFF_SIZE);
    zos::status("{:#04x} {:#04x} {:#04x}",buff_str[0],buff_str[1],buff_str[2]);
    if (buff != NULL) {
        std::copy(buff_str.begin(), buff_str.begin()+UART_BUFF_SIZE, buff);
    }
}

std::vector<int> devicez::read_nano_uart() {
    uint8_t nano_buff[3] = {0};
    std::vector<int> nano_pack(3, 0);
    int infrare_flag;
    int cap_vol;
    int bat_vol_10x;
    std::scoped_lock lock(mutex_uart);
    read_uart(nano_buff);
    if((nano_buff[0] & 0xfa) == 0xfa) {
        // infrare_flag = nano_buff[0] & 0x01;
        // cap_vol = nano_buff[1];
        // bat_vol_10x = nano_buff[2];

        // nano_pack[0] = infrare_flag;
        // nano_pack[1] = cap_vol;
        // nano_pack[2] = bat_vol_10x;
        nano_pack[0] = nano_buff[0];
        nano_pack[1] = nano_buff[1];
        nano_pack[2] = nano_buff[2];
    }else if(nano_buff[0] == 0xaa){
        nano_pack[0] = nano_buff[0];
        nano_pack[1] = nano_buff[1];
        nano_pack[2] = nano_buff[2];
    }
    // else {
    //     zos::log("wrong pack: {:#04x}\n", nano_buff[0]);
    // }
    return nano_pack;
}
int devicez::read_imu(mraa::Uart* uart) {
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
                imu_failures++;
                
            }
        } else {
            std::cout<<"imu acc not found"<<std::endl;
            imu_failures++;
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
                imu_failures++;
            }
        } else {
           std::cout<<"imu acc not found"<<std::endl;
           imu_failures++;
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
                imu_failures++;
            }
        } else {
            std::cout<<"imu acc not found"<<std::endl;
            imu_failures++;
        }
    }
    //std::for_each(data.begin(), data.end(), [](uint8_t element) {
  // 处理每个元素
//   zos::status("=-------------------imudata:{:#04x}\n",element);
//});
    zos::status("=-------------------imu_failures:{}\n",imu_failures);
    if(imu_failures>5000){
        reset=true;
         zos::warning("uart is crashed !");
        // std::ofstream flag_file("restart_flag.txt");
        // if (flag_file.is_open()) {
        //         zos::warning("uart is crashed ,Created restart flag file!");
        //         flag_file.close();
        //     } else {
        //         zos::warning("uart is crashed ,filed to creat flag file!");
        //     }
            imu_failures=0;
    }
    return length;
}


int devicez::read_imu_raw(mraa::Uart* uart1) {
    std::scoped_lock lock(mutex_uart);
    std::string buff_str = uart1->readStr(IMU_DATA_LENGTH);
    int length=buff_str.size();


    if(length > IMU_DATA_LENGTH) length= IMU_DATA_LENGTH;

    if(length==IMU_DATA_LENGTH){
        std::cout<<"get status num: "<<IMU_DATA_LENGTH <<std::endl;
    }
    else if(length<IMU_DATA_LENGTH){
        std::cout<<"get status num: "<<length<<std::endl;
    }
    std::copy(buff_str.begin(), buff_str.begin()+IMU_DATA_LENGTH, buffer);
    
    // printHex(buffer, length);
    return length;
}

bool devicez::sumcrc(uint8_t* data) {
            uint16_t sum = 0x0;
            for (size_t i = 0; i < 10; i++) {
                sum += data[i];  // 累加数据
            }
            uint8_t crc = sum & 0xFF;  // 取校验和的低8位
            // ESP_LOGD("imu crc", "sumcrc: %d, real: %d", crc, data[10]);
            return (crc == data[10]);
        }
