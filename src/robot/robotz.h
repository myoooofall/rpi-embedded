#ifndef ROBOT_H
#define ROBOT_H

#include <thread>

// #ifdef OLD_VERSION
#include "nrf2401.h"
// #else
#include "wifiz.h"
// #endif

#include "robot_comm.pb.h"

#ifdef ROCKPIS_VERSION
    #include "device_ROCKS.h"
#elif defined(CM4_VERSION)
    // #include "device_CM4.h"
    #include "device_pigpio.h"
#endif

#include <yaml-cpp/yaml.h>
#include "zos/core.h"
#include "zss_cmd.pb.h"

class robotz {
public:
    robotz(int _comm_type,int motor_num=4); // _comm_type : 1-24L01,2-wifi
    ~robotz() = default;
    uint8_t robot_id = config::robot_id;
    int nrf2401_freq;
    devicez gpio_devices;
    // #ifdef OLD_VERSION
    comm_2401 nrf2401;
    // #else
    wifi_comm wifiz;
    // #endif
private:
    void _wifi_cb(const asio::ip::udp::endpoint&,const void*,size_t);
    void _send_status_thread(std::stop_token);
    void _multicast_info_thread(std::stop_token);
    void _control_thread(std::stop_token);
    void _gpio_thread(std::stop_token);
    void _cmd_cb(const zos::Data&);
    void _update_status(const double); // delta_t (ms)
    zos::Data _cmd_data;
    std::mutex _cmd_data_mutex;
    zos::Publisher _cmd_publisher;
    zos::Subscriber<2> _cmd_subsciber;
public:
    double ms_since_last_cmd_pack(){ return _time_since_last_pack; }
public:
    void testmode_on();

    static constexpr int COMM_TYPE_24L01 = 1;
    static constexpr int COMM_TYPE_WIFI = 2;
    static constexpr int CMD_TYPE_NONE = 0;
    static constexpr int CMD_TYPE_WHEEL = 1;
    static constexpr int CMD_TYPE_VEL = 2;
    static constexpr int CMD_TYPE_POSE = 3;
    static constexpr int CMD_TYPE_CHASE = 4;
    struct Robot_CMD {
        int comm_type; // 1:24L01,2:WIFI
        int cmd_type; // check zss_cmd.proto (1:CMD_WHEEL,2:CMD_VEL,3:CMD_POSE,4:CMD_CHASE)
        bool need_report; // use in 24L01

        bool kick_en; // if kick enabled
        bool kick_mode;    //chip:1  shoot:0
        float desire_power; // speed(m/s) for flatkick or distance(m) for chip
        float kick_discharge_time = 0; // us
        float drib_power; // -1 ~ 1

        // cmd_wheel
        std::array<float, 4> motor_vel_target = {0,0,0,0}; // rad/s
        // cmd_vel
        float vx_target = 0, vy_target = 0, vr_target = 0;  // robot velocity: cm/s
        bool use_dir = 0;
        //// use to store
        float vx_target_last = 0, vy_target_last = 0, vr_target_last = 0;
        // cmd_pose, cmd_chase
        // TODO

        // old protocol
        uint8_t acc_set = 150;  //加速度限制 16ms内合成加速度最大值，单位cm/s
        uint16_t acc_r_set = 60;
        
        uint8_t DEC_FRAME = 0;
        std::array<float, 4> pid_kp = {0,0,0,0};
        std::array<float, 4> pid_ki = {0,0,0,0};
        std::array<float, 4> pid_kd = {0,0,0,0};
        bool imu_is_used = 0 ;

        // store origin pb cmd
    } robot_cmd;
    ZSS::New::Robot_Command pb_cmd;
    std::mutex _robot_cmd_mutex;

    struct Robot_STATUS {
        int id;
        int team = 0; // 1 for blue,2 for yellow

        // use in 24L01
        int left_report_pack_count = 0;
        int transmitted_pack_count = 0; // check if send2master package normal

        float robot_is_infrared = -1;      // infrared detected time (ms) or -1
        float robot_is_boot_charged;  // if capacitance charge finished (60v)
        float last_kick_time = 10000.0; // ms [0 ~ 10000]
        float robot_is_shooted = 10000.0; // ms
        float robot_is_chipped = 10000.0; // ms

        float cap_vol; // (v)
        float bat_vol; // (v)
        std::array<float, 4> motor_encoder = {0,0,0,0}; // [-4096~4095]
        
        float imu_theta; // suppose to be (-pi,pi]
        std::array<float, 6> imu_info; // TODO
        std::array<float,12> imu_data;

        // uint8_t robot_status = 0, last_robot_status = 0;
    } robot_status;
    std::atomic<double> _time_since_last_pack = 0; // ms
    std::mutex _robot_status_mutex;

    uint8_t Robot_Status = 0, Last_Robot_Status = 0;
    uint8_t Kick_Count = 0;
    int8_t Left_Report_Package = 0;

    // uint8_t RX_Packet[25];
    void set_pid();
    void set_pid_time();
    void set_pid_time1();

    void run_per_13ms(std::stop_token);
    bool get_new_pack();
    void stand();

    void period_test();

    // self test 
    void self_test();
private:
    void test_move(int Vx, int Vy, int Vr);
    void test_dribble(int d_power);
    void test_kick(int shoot_or_chip, int boot_power);
private:
    YAML::Node config_yaml;
    std::atomic_bool pid_busy = false;  // TODO: ?
    robot_comm::Robot comm_pack; 
    std::vector<int> vel_pack = {0,0,0,0};
    std::array<std::atomic<float>,4> vel_read_temp = {0,0,0,0};
    std::vector<int> pid_pack = {0,0,0,0,0,0,0,0};
    std::vector<int> pid_real = {0,0,0,0,0,0,0,0};
    std::vector<uint8_t> TX_Packet = {
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,  //[0-8]
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5,        //[9-16]
        0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5};       //[17-24]
    
    bool _infrared = 0;
    float _cap_vol; // (v)
    float _bat_vol; // (v)
    bool _shooted;
    bool _chipped;

    // ThreadPool thpool;
    std::jthread _jthread4control,_jthread4multicast,_jthread4sendback;
    std::jthread _jthread4gpio;

    int test_charge_count = 0;

    std::chrono::time_point<std::chrono::steady_clock> lasttime;

    // int infr_count = 0;
    bool valid_pack = 0;
    
    const double Vel_k2 = config::vel_ratio;

    static constexpr double sin_angle[4] = {sin(config::car_angle_front), -sin(config::car_angle_front), -sin(config::car_angle_back), sin(config::car_angle_back)};
    static constexpr double cos_angle[4] = {-cos(config::car_angle_front), -cos(config::car_angle_front), cos(config::car_angle_back), cos(config::car_angle_back)};
    
    void pack(std::vector<uint8_t> &TX_Packet);
    int unpack(uint8_t *Packet);
    // void unpack_proto(const void* ptr, size_t size);
    void motion_planner(const double _dt);// _dt in us
    void shoot_chip();
    
    int infrare_detect();
    void infrare_toggin();

    void read_config_yaml();

    void save_pid();
    void save_id(int robot_id_new);
    void save_freq(int nrf2401_freq_new);
    

};

#endif