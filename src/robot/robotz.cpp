#include <fstream>
#include "robotz.h"
using PB = ZSS::New::Robot_Command;
robotz::robotz(int _comm_type,int motor_num)
    : gpio_devices(motor_num)
    , wifiz(std::bind(&robotz::_wifi_cb,this,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3))
    , _cmd_subsciber("cmd",std::bind(&robotz::_cmd_cb,this,std::placeholders::_1))
    , _cmd_publisher("cmd"){

    // Motor Init
    // TODO: change to while
    int motors_num = gpio_devices.motors_detect();
    if ( motors_num == 0 ) {
        zos::log("NO motor detected\n");
    }else {
        zos::log("{} motor detected!\n", motors_num);
    }
    read_config_yaml();
    nrf2401.set_freq(nrf2401_freq);
    set_pid();
    
    // bangbang.control_init();
    // #ifdef OLD_VERSION
    // comm.start();
    // #endif

    // thpool.enqueue(&robotz::run_per_13ms, this);
    if(_comm_type == COMM_TYPE_24L01){
        _jthread4control = std::jthread(std::bind(&robotz::run_per_13ms,this,std::placeholders::_1));
    }else if(_comm_type == COMM_TYPE_WIFI){
        _jthread4control = std::jthread(std::bind(&robotz::_control_thread,this,std::placeholders::_1));
        _jthread4multicast = std::jthread(std::bind(&robotz::_multicast_info_thread,this,std::placeholders::_1));
        _jthread4sendback = std::jthread(std::bind(&robotz::_send_status_thread,this,std::placeholders::_1));
    }
    _jthread4gpio = std::jthread(std::bind(&robotz::_gpio_thread,this,std::placeholders::_1));
}

//解包，到每个轮子的速度
int robotz::unpack(uint8_t *Packet) {
    // #ifdef OLD_VERSION
    // std::scoped_lock lock(nrf2401.mutex_comm_2401);
    // #else
    // std::scoped_lock lock(mutex_comm);
    // #endif
    // if ((Packet[0] & 0xf0) == 0x40)
    std::scoped_lock lock{_robot_cmd_mutex};

    if ((Packet[0] & 0xf0) == 0x40) {
        if ((robot_id == (Packet[1] & 0x0f)) && (Packet[0] & 0x08)) {
            valid_pack = 1;
            robot_cmd.vx_target = (Packet[2] & 0x7f) + ((Packet[17] & 0xc0) << 1);
            robot_cmd.vx_target = (Packet[2] & 0x80) ? ( -robot_cmd.vx_target ) : robot_cmd.vx_target;
            robot_cmd.vx_target = robot_cmd.vx_target / 100.0;
            robot_cmd.vy_target = (Packet[3] & 0x7f) + ((Packet[17] & 0x30) << 3);
            robot_cmd.vy_target = (Packet[3] & 0x80) ? ( -robot_cmd.vy_target ) : robot_cmd.vy_target;
            robot_cmd.vy_target = robot_cmd.vy_target / 100.0;
            robot_cmd.vr_target = (Packet[4] & 0x7f) + ((Packet[17] & 0x0f) << 7);
            robot_cmd.vr_target = (Packet[4] & 0x80) ? ( -robot_cmd.vr_target ) : robot_cmd.vr_target;
            robot_cmd.vr_target = robot_cmd.vr_target / 100.0;
            robot_cmd.need_report = Packet[1] >> 7;
            robot_cmd.drib_power = (Packet[1] >> 4) & 0x03;
            robot_cmd.kick_discharge_time = (Packet[21] & 0x7f) * 50;
            robot_cmd.kick_en = (robot_cmd.kick_discharge_time > 0) ? true : false;
            robot_cmd.kick_mode = ( Packet[1] >> 6 ) & 0x01;
            robot_cmd.use_dir = Packet[21] & 0x80;
        }else if ((robot_id == (Packet[5] & 0x0f)) && (Packet[0] & 0x04)) {
            valid_pack = 1;
            robot_cmd.vx_target = (Packet[6] & 0x7f) + ((Packet[18] & 0xc0) << 1);
            robot_cmd.vx_target = (Packet[6] & 0x80) ? ( -robot_cmd.vx_target ) : robot_cmd.vx_target;
            robot_cmd.vx_target = robot_cmd.vx_target / 100.0;
            robot_cmd.vy_target = (Packet[7] & 0x7f) + ((Packet[18] & 0x30) << 3);
            robot_cmd.vy_target = (Packet[7] & 0x80) ? ( -robot_cmd.vy_target ) : robot_cmd.vy_target;
            robot_cmd.vy_target = robot_cmd.vy_target / 100.0;
            robot_cmd.vr_target = (Packet[8] & 0x7f) + ((Packet[18] & 0x0f) << 7);
            robot_cmd.vr_target = (Packet[8] & 0x80) ? ( -robot_cmd.vr_target ) : robot_cmd.vr_target;
            robot_cmd.vr_target = robot_cmd.vr_target / 100.0;
            robot_cmd.need_report = Packet[5] >> 7;
            robot_cmd.drib_power = (Packet[5] >> 4) & 0x03;
            robot_cmd.kick_discharge_time = (Packet[22] & 0x7f) * 50;
            robot_cmd.kick_en = (robot_cmd.kick_discharge_time > 0) ? true : false;
            robot_cmd.kick_mode = ( Packet[5] >> 6 ) & 0x01;
            robot_cmd.use_dir = Packet[22] & 0x80;
        }else if ((robot_id == (Packet[9] & 0x0f)) && (Packet[0] & 0x02)) {
            valid_pack = 1;
            robot_cmd.vx_target = (Packet[10] & 0x7f) + ((Packet[19] & 0xc0) << 1);
            robot_cmd.vx_target = (Packet[10] & 0x80) ? ( -robot_cmd.vx_target ) : robot_cmd.vx_target;
            robot_cmd.vx_target = robot_cmd.vx_target / 100.0;
            robot_cmd.vy_target = (Packet[11] & 0x7f) + ((Packet[19] & 0x30) << 3);
            robot_cmd.vy_target = (Packet[11] & 0x80) ? ( -robot_cmd.vy_target ) : robot_cmd.vy_target;
            robot_cmd.vy_target = robot_cmd.vy_target / 100.0;
            robot_cmd.vr_target = (Packet[12] & 0x7f) + ((Packet[19] & 0x0f) << 7);
            robot_cmd.vr_target = (Packet[12] & 0x80) ? ( -robot_cmd.vr_target ) : robot_cmd.vr_target;
            robot_cmd.vr_target = robot_cmd.vr_target / 100.0;
            robot_cmd.need_report = Packet[9] >> 7;
            robot_cmd.drib_power = (Packet[9] >> 4) & 0x03;
            robot_cmd.kick_discharge_time = (Packet[23] & 0x7f) * 50;
            robot_cmd.kick_en = (robot_cmd.kick_discharge_time > 0) ? true : false;
            robot_cmd.kick_mode = ( Packet[9] >> 6 ) & 0x01;
            robot_cmd.use_dir = Packet[23] & 0x80;
        }else if ((robot_id == (Packet[13] & 0x0f)) && (Packet[0] & 0x01)) {
            valid_pack = 1;
            robot_cmd.vx_target = (Packet[14] & 0x7f) + ((Packet[20] & 0xc0) << 1);
            robot_cmd.vx_target = (Packet[14] & 0x80) ? ( -robot_cmd.vx_target ) : robot_cmd.vx_target;
            robot_cmd.vx_target = robot_cmd.vx_target / 100.0;
            robot_cmd.vy_target = (Packet[15] & 0x7f) + ((Packet[20] & 0x30) << 3);
            robot_cmd.vy_target = (Packet[15] & 0x80) ? ( -robot_cmd.vy_target ) : robot_cmd.vy_target;
            robot_cmd.vy_target = robot_cmd.vy_target / 100.0;
            robot_cmd.vr_target = (Packet[16] & 0x7f) + ((Packet[20] & 0x0f) << 7);
            robot_cmd.vr_target = (Packet[16] & 0x80) ? ( -robot_cmd.vr_target ) : robot_cmd.vr_target;
            robot_cmd.vr_target = robot_cmd.vr_target / 100.0;
            robot_cmd.need_report = Packet[13] >> 7;
            robot_cmd.drib_power = (Packet[13] >> 4) & 0x03;                                      
            robot_cmd.kick_discharge_time = (Packet[24] & 0x7f) * 50;
            robot_cmd.kick_en = (robot_cmd.kick_discharge_time > 0) ? true : false;
            robot_cmd.kick_mode = ( Packet[13] >> 6 ) & 0x01;
            robot_cmd.use_dir = Packet[24] & 0x80;
        }else {
            valid_pack = 0;
            // zos::warning("pack[1] info: {}\n", Packet[1] & 0x0f);
        }
        // gpio_devices.led_flash();
        // TODO: rxbuf receive test
        // zos::info("robot_id: {}        vx: {}   vy: {}   vr: {}\n", robot_id, robot_cmd.vx_target, robot_cmd.vy_target, robot_cmd.vr_target);
    }else if (Packet[0] == 0xab) {
        // pid_pack[0] = 0xf5;
        // pid_pack[1] = Packet[1];
        // pid_pack[2] = Packet[2];
        if(Packet[1] >> 7 == 1) {
            pid_pack[0] = Packet[2];
            pid_pack[1] = Packet[3];
        }
        if(Packet[5] >> 7 == 1) {
            pid_pack[2] = Packet[6];
            pid_pack[3] = Packet[7];
        }
        if(Packet[9] >> 7 == 1) {
            pid_pack[4] = Packet[10];
            pid_pack[5] = Packet[11];
        }
        if(Packet[13] >> 7 == 1) {
            pid_pack[6] = Packet[14];
            pid_pack[7] = Packet[15];
        }
        
        robot_cmd.need_report = 1;
        valid_pack = 1;
        zos::info("p: {}   i: {}\n", pid_pack[0], pid_pack[1]);
        set_pid();
    }else {
        valid_pack = 0;
        zos::error("wrong pack: {} {} {}\n", Packet[0], Packet[1], Packet[2]);
    }
    
    return valid_pack;
}

// bool robotz::unpack_proto(const void* ptr, size_t size) {
//     bool valid = false;
//     // valid = comm_pack.ParseFromString(proto_string);
//     // // zos::info("parsing protobuf\n");
//     // if (valid) {
//     //     zos::info("Proto test robot_cmd.vx_target: {}\n", comm_pack.robot_cmd.vx_target());
//     //     zos::info("Proto test robot_cmd.vy_target: {}\n", comm_pack.robot_cmd.vy_target());
//     // }
//     return valid;
// }
void robotz::_wifi_cb(const asio::ip::udp::endpoint& ep,const void* ptr,size_t size){
    wifiz.set_master_ip(ep.address().to_string());
    {
        std::scoped_lock lock{_cmd_data_mutex};
        _cmd_data.resize(size);
        _cmd_data.store(ptr,size);
    }

    _cmd_publisher.publish();
}
void robotz::_cmd_cb(const zos::Data& data){
    std::scoped_lock lock{_cmd_data_mutex};
    auto res = pb_cmd.ParseFromArray(_cmd_data.data(),_cmd_data.size());
    if(res){
        
        if(gpio_devices.reset){
            
            robot_cmd.cmd_type = CMD_TYPE_VEL;
            robot_cmd.kick_en = false;
            robot_cmd.kick_discharge_time = 0;
            robot_cmd.vy_target = 0;
            robot_cmd.vx_target = 0;
            robot_cmd.vr_target = 0;
            robot_cmd.use_dir = 0;
            robot_cmd.drib_power = 0;
            std::fill_n(begin(vel_pack), MAX_MOTOR, 0);
            zos::status("robot stand!");
           

        }
        else{
        auto& _p = pb_cmd;
        auto& _c = robot_cmd;
        _time_since_last_pack = 0;
        zos::log("{}\n",_p.DebugString());
        // TODO update
        std::scoped_lock lock{_robot_cmd_mutex};
        // _c.id = _p.robot_id(); // maybe need to check if id correct
        _c.kick_en = (_p.kick_mode() != PB::NONE);
        _c.kick_mode = (_p.kick_mode() == PB::CHIP);
        _c.desire_power = _p.desire_power();
        _c.kick_discharge_time = _p.kick_discharge_time();
        _c.drib_power = _p.dribble_spin();
        _c.cmd_type = int(_p.cmd_type());
        switch(_p.cmd_type()){
            case PB::CMD_WHEEL:{
                _c.motor_vel_target[0] = _p.cmd_wheel().wheel1();
                _c.motor_vel_target[1] = _p.cmd_wheel().wheel2();
                _c.motor_vel_target[2] = _p.cmd_wheel().wheel3();
                _c.motor_vel_target[3] = _p.cmd_wheel().wheel4();
                break;
            }
            case PB::CMD_VEL:{
                _c.vx_target = _p.cmd_vel().velocity_x();
                _c.vy_target = _p.cmd_vel().velocity_y();
                _c.use_dir = _p.cmd_vel().use_imu();
                _c.vr_target = _p.cmd_vel().velocity_r();
                break;
            }
            default:{
                zos::error("current cmd_type not supported:{}\n",_p.cmd_type());
                break;
            }
        }
        
    }
    }
}
void robotz::_update_status(const double dt){
    std::vector<int> nano_pack = gpio_devices.read_nano_uart();
    {
        std::scoped_lock lock{_robot_status_mutex};
        auto& _s = robot_status;
        if (nano_pack[0] == 0xaa) {
            int freq_temp = nano_pack[1];
            int id_temp = nano_pack[2];
            if (id_temp != _s.id) {
                _s.id = id_temp;
                save_id(id_temp);
                gpio_devices.buzzer_set_freq_num();
                zos::warning("change num: {}\n", id_temp);
            }
            if(freq_temp == 6 && _s.team != 2) {
                _s.team = 2;
		save_freq(freq_temp);
                gpio_devices.buzzer_set_freq_num();
            }else if(freq_temp == 8 && _s.team != 1) {
                _s.team = 1;
		save_freq(freq_temp);
                gpio_devices.buzzer_set_freq_num();
            }
        }else if((nano_pack[0] & 0xfa) == 0xfa) {
            zos::status("nano pack 0: {:#}\n", nano_pack[0]);
            if((nano_pack[0] & 0x01)) {
                if(_s.robot_is_infrared > 0){
                    _s.robot_is_infrared = std::min(_s.robot_is_infrared+dt,10000.0);
                }else{
                    _s.robot_is_infrared = 1;
                }
                zos::status("robot_is_infrared: {}ms\n", _s.robot_is_infrared);
            }else if (_s.robot_is_infrared < 0) {
                _s.robot_is_infrared = std::max(_s.robot_is_infrared-dt,-10000.0);
            }else {
                _s.robot_is_infrared = -1;
            }
            _s.cap_vol = nano_pack[1];
            _s.bat_vol = nano_pack[2]/10.0;
            zos::status("infrare: {}, cap_vol : {}, bat_vol: {}\n", _s.robot_is_infrared, _s.cap_vol, _s.bat_vol);
        }
        // nano uart and kick status will update in gpio thread
        // if(_infrared) {
        //     _s.robot_is_infrared += dt;
        // }else{
        //     _s.robot_is_infrared = -1;
        // }
        // _s.bat_vol = _bat_vol;
        // _s.cap_vol = _cap_vol;

        // motors will be updated in control thread;
        for(int i=0;i<MAX_MOTOR;i++){
            _s.motor_encoder[i] = vel_read_temp[i];
        }
        zos::warning("motor get : {},{},{},{}\n",_s.motor_encoder[0],_s.motor_encoder[1],_s.motor_encoder[2],_s.motor_encoder[3]);
        // kick status will be reset to 0 in control thread (no closed loop check)
        _s.last_kick_time = std::min(_s.last_kick_time+dt,10000.0);
        _s.robot_is_shooted = std::min(_s.robot_is_shooted+dt,10000.0);
        _s.robot_is_chipped = std::min(_s.robot_is_chipped+dt,10000.0);
        
        int length=gpio_devices.read_imu(gpio_devices.uart1);
        zos::status("i----------------mu_data num: {}-----------------\n", length);
        _s.imu_data[0]= gpio_devices.imu_status.acc_x;
        _s.imu_data[1]= gpio_devices.imu_status.acc_y;
        _s.imu_data[2]= gpio_devices.imu_status.acc_z;
        _s.imu_data[3]= gpio_devices.imu_status.T_degree;
        _s.imu_data[4]= gpio_devices.imu_status.omega_x;
        _s.imu_data[5]= gpio_devices.imu_status.omega_y;
        _s.imu_data[6]= gpio_devices.imu_status.omega_z;
        _s.imu_data[7]= gpio_devices.imu_status.voltage;
        _s.imu_data[8]= gpio_devices.imu_status.theta_x;
        _s.imu_data[9]= gpio_devices.imu_status.theta_y;
        _s.imu_data[10]= gpio_devices.imu_status.theta_z;
        _s.imu_data[11]= gpio_devices.imu_status.version;
    }
}
void robotz::_send_status_thread(std::stop_token _stop_token){
    zos::Rate robot_rate(config::sendback_freq);
    zos::Data data;
    ZSS::New::Robot_Status pb_status;
    while(true){
        if(_stop_token.stop_requested()) break;
        _update_status(1000.0/config::sendback_freq);
        if(wifiz.get_master_ip() != ""){
            break;
        }
        robot_rate.sleep();
    }
    while(true){
        if(_stop_token.stop_requested()) break;
        // TODO
        _update_status(1000.0/config::sendback_freq); // ms
        {
            std::scoped_lock lock{_robot_status_mutex};
            auto& _s = robot_status;
            pb_status.set_robot_id(_s.id);
            pb_status.set_infrared(_s.robot_is_infrared);
            // zos::status("id: {}, team: {}\n", _s.id, _s.team);
            zos::status("send infrare status: {}\n", _s.robot_is_infrared);
            pb_status.set_flat_kick(_s.robot_is_shooted);
            pb_status.set_chip_kick(_s.robot_is_chipped);
            pb_status.set_battery(_s.bat_vol);
            pb_status.set_capacitance(_s.cap_vol);
            pb_status.add_wheel_encoder(_s.motor_encoder[0]);
            pb_status.add_wheel_encoder(_s.motor_encoder[1]);
            pb_status.add_wheel_encoder(_s.motor_encoder[2]);
            pb_status.add_wheel_encoder(_s.motor_encoder[3]);
            pb_status.set_team(ZSS::New::Team(_s.team));
            pb_status.add_imu_data(_s.imu_data[0]);
            pb_status.add_imu_data(_s.imu_data[1]);
            pb_status.add_imu_data(_s.imu_data[2]);
            pb_status.add_imu_data(_s.imu_data[3]);
            pb_status.add_imu_data(_s.imu_data[4]);
            pb_status.add_imu_data(_s.imu_data[5]);
            pb_status.add_imu_data(_s.imu_data[6]);
            pb_status.add_imu_data(_s.imu_data[7]);
            pb_status.add_imu_data(_s.imu_data[8]);
            pb_status.add_imu_data(_s.imu_data[9]);
            pb_status.add_imu_data(_s.imu_data[10]);
            pb_status.add_imu_data(_s.imu_data[11]); 
        }
        auto size = pb_status.ByteSizeLong();
        data.resize(size);
        pb_status.SerializeToArray(data.ptr(),size);
        wifiz.udp_sender(data.data(),size);
        pb_status.Clear();
        robot_rate.sleep();
    }
}
void robotz::_multicast_info_thread(std::stop_token _stop_token){
    zos::Rate robot_rate(config::multicast_freq);
    zos::Data data;
    ZSS::New::Multicast_Status mc_status;
    Robot_STATUS _s;
    while(true){
        _time_since_last_pack += 1000.0 / config::multicast_freq; // ms
        {
            std::scoped_lock lock{_robot_status_mutex};
            _s = robot_status;
        }
        if(_stop_token.stop_requested()) break;
        mc_status.set_ip(wifiz.get_ip());
        mc_status.set_uuid("0000-0000-0000-0000");
        mc_status.set_team(ZSS::New::Team(_s.team));
        mc_status.set_robot_id(_s.id); // ROBOT_ID change 
        mc_status.set_battery(_s.bat_vol);
        mc_status.set_capacitance(_s.cap_vol);
        auto size = mc_status.ByteSizeLong();
        data.resize(size);
        mc_status.SerializeToArray(data.ptr(),size);
        wifiz.udp_sender_mc(data.data(),size);
        mc_status.Clear();
        robot_rate.sleep();
    }
}
void robotz::_control_thread(std::stop_token _stop_token){
    int count = 0;
    auto now_time = std::chrono::steady_clock::now();
    auto last_time = now_time - std::chrono::microseconds(1000000)/config::control_cmd_freq;
    zos::Rate robot_rate(config::control_cmd_freq);
    while(true){
        Robot_CMD _c;
        Robot_STATUS _s;
        {
            std::scoped_lock lock{_robot_cmd_mutex};
            _c = robot_cmd;
        }
        {
            std::scoped_lock lock{_robot_status_mutex};
            _s = robot_status;
        }
        {   
            now_time = std::chrono::steady_clock::now();
            switch(_c.cmd_type){
                case CMD_TYPE_NONE:{
                    break;
                }
                case CMD_TYPE_WHEEL:{
                    break;
                }
                case CMD_TYPE_VEL:{
                    motion_planner(std::chrono::duration_cast<std::chrono::microseconds>(now_time - last_time).count());
                    break;
                }
                default:{
                    zos::error("_control_thread check cmd_type not supported\n");
                    break;
                }
            }
            last_time = now_time;
        }
        // zos::info("motor set : {},{},{},{}\n",vel_pack[0],vel_pack[1],vel_pack[2],vel_pack[3]);
        gpio_devices.set_motors_vel(vel_pack);
        // TODO: dribble
        gpio_devices.dribbler((int)abs(_c.drib_power) | 0x0c);
        

        auto temp_read = gpio_devices.get_motors_vel();
        for(int i=0;i<MAX_MOTOR;i++){
            vel_read_temp[i] = temp_read[i];
        }
        robot_rate.sleep();
    }
}

void robotz::_gpio_thread(std::stop_token _stop_token) {
    zos::Rate robot_rate(config::robot_freq);
    while(true){
        // nano
        // std::vector<int> nano_pack = gpio_devices.read_nano_uart();
        // int freq_temp = -1;
        // int id_temp = -1;
        // if (nano_pack[0] == 0xaa) {
        //     freq_temp = nano_pack[1];
        //     id_temp = nano_pack[2];
        //     if (id_temp != _s.id) {
        //         _s.id = id_temp;
        //         save_id(id_temp);
        //         gpio_devices.buzzer_set_freq_num();
        //         zos::warning("change num: {}\n", id_temp);
        //     }
        //     if(freq_temp == 6 && _s.team != 2) {
        //         _s.team = 2;
        //         gpio_devices.buzzer_set_freq_num();
        //     }else if(freq_temp == 8 && _s.team != 1) {
        //         _s.team = 1;
        //         gpio_devices.buzzer_set_freq_num();
        //     }
        // }else if((nano_pack[0] & 0xfa) == 0xfa) {
        //     _infrared = nano_pack[0] & 0x01;
        //     // zos::status("robot_is_infrared: {}ms\n", _s.robot_is_infrared);
        //     _cap_vol = nano_pack[1];
        //     _bat_vol = nano_pack[2] / 10.0;
        //     zos::status("infrare: {}, cap_vol : {}, bat_vol: {}\n", _infrared, _cap_vol, _bat_vol);
        // }
        
        Robot_CMD _c;
        Robot_STATUS _s;
        {
            std::scoped_lock lock{_robot_cmd_mutex};
            _c = robot_cmd;
        }
        {
            std::scoped_lock lock{_robot_status_mutex};
            _s = robot_status;
        }
        // kick
        int kick_status = -1;
        if (_s.robot_is_infrared>0 && _c.kick_en && _s.last_kick_time>300) {
            zos::log("kick info   en:{}, mode:{}, time:{}   ir: {}, last_time: {}\n", _c.kick_en, _c.kick_mode, _c.kick_discharge_time, _s.robot_is_infrared, _s.last_kick_time);
            kick_status = gpio_devices.shoot_chip(_c.kick_mode, _c.kick_discharge_time);
            // TODO: _s.robot_is_shooted/chipped
            {
                std::scoped_lock lock{_robot_status_mutex,_robot_cmd_mutex};
                auto& _status = robot_status;
                switch(kick_status) {
                    case SHOOT_MODE:
                        zos::log("--kick status: shoot!\n");
                        _status.robot_is_shooted = 0;
                        break;
                    case CHIP_MODE:
                        zos::log("--kick status: chip!\n");
                        _status.robot_is_chipped = 0;
                        break;
                }
                robot_status.last_kick_time = 0;
                robot_cmd.kick_en = false;
                robot_cmd.kick_discharge_time = 0;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10000));
        }
        // std::this_thread::sleep_for(std::chrono::microseconds(1000));
        robot_rate.sleep();
    }
}

void robotz::motion_planner(const double _dt) { // dt in us
    std::scoped_lock lock{_robot_cmd_mutex};
    double acc_x = (robot_cmd.vx_target - robot_cmd.vx_target_last)/(_dt/1000000);
    double acc_y = (robot_cmd.vy_target - robot_cmd.vy_target_last)/(_dt/1000000);
    double acc_whole = sqrt(acc_x * acc_x + acc_y * acc_y);
    if (acc_whole > config::a_max) {
        double sin_x = acc_x / acc_whole;
        double sin_y = acc_y / acc_whole;
        acc_whole = config::a_max;
        acc_x = acc_whole * sin_x;
        acc_y = acc_whole * sin_y;
    }
    double vx = 0,vy = 0;
    vx = robot_cmd.vx_target_last + acc_x*_dt/1000000.0;
    vy = robot_cmd.vy_target_last + acc_y*_dt/1000000.0;

    robot_cmd.vx_target_last = vx;
    robot_cmd.vy_target_last = vy;

    double vr= get_vr(_dt);
    double acc_r = (vr - robot_cmd.vr_target_last)/(_dt/1000000);
    if(acc_r > robot_cmd.acc_r_set){
        acc_r = robot_cmd.acc_r_set;
    }
    vr = robot_cmd.vr_target_last + acc_r*_dt/1000000.0;
    robot_cmd.vr_target_last = vr ;

    for(int i=0; i < 4; i++) {
        vel_pack[i] = ((sin_angle[i]) * vx + (cos_angle[i]) * vy - 8.2 * vr/160) * Vel_k2 * 100; // TODO: *100
        vel_pack[i] = (vel_pack[i] >  8000) ?  8000 : vel_pack[i];
        vel_pack[i] = (vel_pack[i] < -8000) ? -8000 : vel_pack[i];
    }

    // zos::info("vel_pack: {} {} {} {}\n", vel_pack[0], vel_pack[1], vel_pack[2], vel_pack[3]);
}

void robotz::stand() {
    std::scoped_lock lock{_cmd_data_mutex};
    robot_cmd.cmd_type = CMD_TYPE_VEL;

    robot_cmd.kick_en = false;
    robot_cmd.kick_discharge_time = 0;

    robot_cmd.vy_target = 0;
    robot_cmd.vx_target = 0;
    robot_cmd.vr_target = 0;
    robot_cmd.use_dir = 0;
    robot_cmd.drib_power = 0;
                    
    std::fill_n(begin(vel_pack), MAX_MOTOR, 0);
    
    // gpio_devices.led_flash();
    #ifndef OLD_VERSION
    wifiz.udp_restart();
    #endif
}

// void robotz::regular() {
//     motion_planner(config::dt_us);
//     if ((Robot_Status != Last_Robot_Status) || (robot_status.robot_is_infrared) || (robot_cmd.need_report == 1)) {
//         Left_Report_Package = 4;
//         Last_Robot_Status = Robot_Status;
//     }
    
//     if(Kick_Count > 0)
//         Kick_Count--;
//     else
//         Robot_Status &= 0xCF;
        
//     if(Left_Report_Package > 0 || Kick_Count > 0){
//         pack(TX_Packet);
//         wifiz.udp_sender(TX_Packet);
//         robot_status.transmitted_pack_count++;
//     }

//     if(Left_Report_Package > 0)   Left_Report_Package --;
// }

void robotz::pack(std::vector<uint8_t> &TX_Packet) {
    int temp_bat;
    int temp_boot;
    
    std::fill_n(begin(TX_Packet), TX_BUF_SIZE, 0);
    TX_Packet[0] = 0xff;
    TX_Packet[1] = 0x02;
    TX_Packet[2] = robot_id;
    TX_Packet[3] = Robot_Status;
    
    int bat_vol_temp = 0;
    int cap_vol_temp = 0;
    {
        std::scoped_lock lock{_robot_status_mutex};
        bat_vol_temp = robot_status.bat_vol * 255 / 10;
        cap_vol_temp = robot_status.cap_vol * 255 / 180;
    }
    TX_Packet[4] = (uint8_t)bat_vol_temp;
    TX_Packet[5] = (uint8_t)cap_vol_temp;
    // zos::status("pack voltage info     bat: {}   cap: {}\n", TX_Packet[4], TX_Packet[5]);

    // TODO: define IMU
    #ifdef IMU_TEST_1
    memcpy(TX_Packet+14, IMU_RX_Buffer+4, 10);
    TX_Packet[24]= IMU_RX_Buffer[2];
    #endif
    
    //TX_Packet[6] = robot_cmd.kick_discharge_time;
    //TX_Packet[7] = robot_status.transmitted_pack_count;
    //uint32_t tick = HAL_GetTick();
    //TX_Packet[8] = tick >> 8 & 0xFF;
    //TX_Packet[9] = tick & 0xFF;
    //TX_Packet[10] = Left_Report_Package;
    // int temp1 = Encoder_count_Motor1_avg;
    // int temp2 = Encoder_count_Motor2_avg;
    // int temp3 = Encoder_count_Motor3_avg;
    // int temp4 = Encoder_count_Motor4_avg;

    // encoder
    robot_status.motor_encoder = gpio_devices.get_motors_vel();
    TX_Packet[6] = ((int)(robot_status.motor_encoder[0]) & 0xFF00)>>8;
    TX_Packet[7] = ((int)(robot_status.motor_encoder[0]) & 0xFF);
    TX_Packet[8] = ((int)(robot_status.motor_encoder[1]) & 0xFF00)>>8;
    TX_Packet[9] = ((int)(robot_status.motor_encoder[1]) & 0xFF);
    TX_Packet[10] = ((int)(robot_status.motor_encoder[2]) & 0xFF00)>>8;
    TX_Packet[11] = ((int)(robot_status.motor_encoder[2]) & 0xFF);
    TX_Packet[12] = ((int)(robot_status.motor_encoder[3]) & 0xFF00)>>8;
    TX_Packet[13] = ((int)(robot_status.motor_encoder[3]) & 0xFF);

    // pid real
    TX_Packet[14] = pid_real[0];
    TX_Packet[15] = pid_real[1];
    TX_Packet[16] = pid_real[2];
    TX_Packet[17] = pid_real[3];
    TX_Packet[18] = pid_real[4];
    TX_Packet[19] = pid_real[5];
    TX_Packet[20] = pid_real[6];
    TX_Packet[21] = pid_real[7];
    // zos::log("tx package: {}\n", TX_Packet[0]);
}

void robotz::run_per_13ms(std::stop_token _stop_token) {
    int status_count = 0;
    zos::Rate robot_rate(config::robot_freq);
    while (true)
    {   
        if(pid_busy){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        if(_stop_token.stop_requested()) break;

        motion_planner(1000000*1/config::robot_freq); // 10 ms
        gpio_devices.set_motors_vel(vel_pack);
        // gpio_devices.charge_switch();

        // dirbble
        // TODO: power -1~1
        // gpio_devices.dribbler(robot_cmd.drib_power);
        {
            std::scoped_lock lock{_cmd_data_mutex};
            gpio_devices.dribbler((int)abs(robot_cmd.drib_power) | 0x0c);
        }


        //infrare
        std::vector<int> nano_pack = gpio_devices.read_nano_uart();
        if (nano_pack[0] == 0xaa) {
            int freq_temp = nano_pack[1];
            int id_temp = nano_pack[2];
            if (freq_temp != nrf2401_freq) {
                nrf2401.set_freq(freq_temp);
                nrf2401_freq = freq_temp;
                save_freq(nrf2401_freq);
                gpio_devices.buzzer_set_freq_num();
                zos::warning("change freq: {}\n", freq_temp);
            }
            if (id_temp != robot_id) {
                robot_id = id_temp;
                save_id(robot_id);
                gpio_devices.buzzer_set_freq_num();
                zos::warning("change num: {}\n", id_temp);
            }
        }else if((nano_pack[0] & 0xfa) == 0xfa) {
            std::scoped_lock lock{_robot_status_mutex};
            if((nano_pack[0] & 0x01)) {
                robot_status.robot_is_infrared = 1;
                Robot_Status = Robot_Status | (1 << 6);
            }else {
                robot_status.robot_is_infrared = 0;
                Robot_Status = Robot_Status & 0x30;
            }
            robot_status.cap_vol = nano_pack[1];
            robot_status.bat_vol = nano_pack[2]/10.0;
            if(status_count++ > 100) {
                zos::status("infrare: {},   cap_vol: {:.1f}V,   bat_vol: {:.1f}V\n", robot_status.robot_is_infrared, robot_status.cap_vol, robot_status.bat_vol);
                status_count = 0;
            }
        }
        {
            std::scoped_lock lock{_robot_status_mutex};
            if(robot_status.last_kick_time < 10000) {
                robot_status.last_kick_time += 1000*1/config::robot_freq;
            }
        }

        // shoot and chip
        // TODO: Shoot and chip-need to determine time flag
        // if(chipshoot_timerdelay_flag < 1000)
        //     chipshoot_timerdelay_flag++;
        // {
        //     std::scoped_lock lock{_cmd_data_mutex};
        //     if( robot_cmd.need_report ) {
        //         pack(TX_Packet);
        //         // #ifdef OLD_VERSION
        //         // zos::info("ready to send\n");
        //         nrf2401.send(std::data(TX_Packet));
        //         // #else
        //         // wifiz.udp_sender(TX_Packet);
        //         // #endif
        //         robot_status.transmitted_pack_count++;
        //     }
        // }

        robot_rate.sleep();
        // period_test();
    }
}

bool robotz::get_new_pack() {
    if (unpack(rxbuf)) {
        // Correct package

        if(Kick_Count > 0) {
            Kick_Count--;
        }else {
            Robot_Status &= 0xCF;
        }
        {
            std::scoped_lock lock{_robot_status_mutex, _cmd_data_mutex};
            if(robot_cmd.kick_discharge_time>0) {
                robot_cmd.kick_en = true;
                Kick_Count = 9;
            }
            if ((Robot_Status != Last_Robot_Status) || (robot_status.robot_is_infrared) || (robot_cmd.need_report == 1)) {
                Left_Report_Package = 6;
                Last_Robot_Status = Robot_Status;
            }
            // zos::status("pack infrare before kick: {}\n", (Robot_Status >> 6));
        }

        if(Left_Report_Package > 0 || Kick_Count > 0) {
            pack(TX_Packet);
            if((TX_Packet[3]&0x40) == 0x40) {
                zos::log("pack infrare triggered\n");
            }
            // #ifdef OLD_VERSION
            // zos::info("ready to send\n");
            nrf2401.send(std::data(TX_Packet));
            // #else
            // wifiz.udp_sender(TX_Packet.data(),TX_Packet.size());
            // #endif
            {
                std::scoped_lock lock{_robot_status_mutex};
                robot_status.transmitted_pack_count++;
            }
        }

        if(Left_Report_Package > 0)   Left_Report_Package --;
    }

    Received_packet = false;
    return valid_pack;
}

void robotz::period_test() {
    auto currenttime = std::chrono::steady_clock::now();
    auto step = (currenttime - lasttime);
    zos::info("period time: {}\n", step.count()/1000);
    lasttime = currenttime;
}

void robotz::testmode_on() {
    gpio_devices.output_test();
}

void robotz::self_test() {
    test_move(20, 0, 0);
    test_move(0, 20, 0);
    test_move(0, 0, 60);
    
    test_dribble(1);
    test_dribble(3);

    // test_kick(0, 40);
    // test_kick(1, 40);
}

void robotz::test_move(int Vx, int Vy, int Vr) {
    robot_cmd.vx_target = Vx;
    robot_cmd.vy_target = Vy;
    robot_cmd.vr_target = Vr;
    motion_planner(config::dt_us);
    zos::log("move x:{} y:{} z:{}\n", Vx, Vy, Vr);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot_cmd.vx_target = 0;
    robot_cmd.vy_target = 0;
    robot_cmd.vr_target = 0;
    motion_planner(config::dt_us);
}
void robotz::test_dribble(int d_power) {
    robot_cmd.drib_power = d_power;
    zos::log("dribble {}\n", d_power);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot_cmd.drib_power = 0;
}
void robotz::test_kick(int shoot_or_chip, int boot_power) {
    robot_cmd.kick_mode = shoot_or_chip;
    robot_cmd.kick_discharge_time = boot_power;
    zos::log("shoot/chip:{}  power:{}\n", shoot_or_chip, boot_power);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    robot_cmd.kick_discharge_time = 0;
}

void robotz::read_config_yaml() {
    try {
        config_yaml = YAML::LoadFile("config.yaml");
        // id
        robot_id = config_yaml["id"].as<int>();
        zos::log("robot id: {}\n", robot_id);
        {
            std::scoped_lock lock{_robot_status_mutex};
            auto& _s = robot_status;
            _s.id = robot_id;
        }
        // freq
        nrf2401_freq = config_yaml["freq"].as<int>();
        zos::log("nrf2401 freq: {}\n", nrf2401_freq);
        // team: 1 for blue,2 for yellow
        {
            std::scoped_lock lock{_robot_status_mutex};
            auto& _s = robot_status;
            if(nrf2401_freq == 6) {
                _s.team = 2;
            }else if(nrf2401_freq == 8) {
                _s.team = 1;
            }
        }
        zos::warning("id: {}, team: {}\n", robot_status.id, robot_status.team);

        // motors pid
        for(auto motor : config_yaml["motor_pid"]) {
            int motor_id = motor["id"].as<int>();
            zos::log("motor{}  p: {}  i: {}\n", motor_id, motor["p"].as<int>(), motor["i"].as<int>());
            pid_pack[motor_id*2] = motor["p"].as<int>();
            pid_pack[motor_id*2+1] = motor["i"].as<int>();
        }
    }catch(const YAML::BadFile& e) {
        std::cerr << e.msg << std::endl;
    }catch(const YAML::ParserException& e) {
        std::cerr << e.msg << std::endl;
    }catch(...) {
        zos::error("read config.yaml failed, use default id #{}\n", robot_id);
    }
}

void robotz::save_id(int robot_id_new) {
    if (config_yaml.IsNull()) {
        config_yaml = YAML::LoadFile("config.yaml");
    }
    config_yaml["id"] = robot_id_new;
    std::ofstream file_stream("config.yaml");
    file_stream << config_yaml; // dump it back into the file
    file_stream.close();
    zos::log("robot id save\n");
}

void robotz::save_freq(int nrf2401_freq_new) {
    if (config_yaml.IsNull()) {
        config_yaml = YAML::LoadFile("config.yaml");
    }
    config_yaml["freq"] = nrf2401_freq_new;
    std::ofstream file_stream("config.yaml");
    file_stream << config_yaml; // dump it back into the file
    file_stream.close();
    zos::log("robot id save\n");
}

void robotz::save_pid() {
    if (config_yaml.IsNull()) {
        config_yaml = YAML::LoadFile("config.yaml");
    }
    for(auto motor : config_yaml["motor_pid"]) {
        int motor_id = motor["id"].as<int>();
        if (pid_real[motor_id*2] > 0) {
            motor["p"] = pid_real[motor_id*2];
        }
        if (pid_real[motor_id*2+1] > 0) {
            motor["i"] = pid_real[motor_id*2+1];
        }
    }
    std::ofstream file_stream("config.yaml");
    file_stream << config_yaml; // dump it back into the file
    file_stream.close();
    zos::log("pid update\n");
    // pid_read();
}

void robotz::set_pid() {
    stand();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    pid_busy = true;
    gpio_devices.set_motors_pid(pid_pack);
    pid_real = gpio_devices.get_motors_pid();
    save_pid();
    std::fill_n(begin(pid_pack), 8, 0);
    pid_busy = false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

double robotz::get_vr(const double _dt){
    
    if(!robot_cmd.use_dir) return robot_cmd.vr_target;
    float current_angle= gpio_devices.imu_status.theta_z*2*3.1415926/360;
    float target_angle= robot_cmd.vr_target;
    error = get_angle(target_angle - current_angle);
    float derivative = (error - prev_error) /_dt;
    prev_error = error ;
    double output_vr = robot_cmd.angle_pid[0] * error + robot_cmd.angle_pid[2] * derivative;
    zos::status("output_vr: {} error: {} target_theta: {} current angle: {}\n",output_vr,error,target_angle,current_angle);
    zos::status("angle_p:{} ,angle_d:{}",robot_cmd.angle_pid[0],robot_cmd.angle_pid[2]);
    return output_vr;

}
double robotz::get_angle(float angle){
    if(angle<-3.1415926) return 2*3.1415926+angle;
    else if(angle>3.1415926) return angle-2*3.1415926;
    else return angle;
}