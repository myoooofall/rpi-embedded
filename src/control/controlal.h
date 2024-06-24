#ifndef CONTROLAL_H
#define CONTROLAL_H

#include <thread>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <mutex>
#include "sensor.h"
#include "config.h"
#include "zos/utils/rate.h"

// class rate{
// private:
//     std::chrono::time_point<std::chrono::steady_clock> start;
//     std::chrono::duration<int, std::nano>expected_cycle_time_;
// public:
//     rate(int frequency){
//     start = std::chrono::steady_clock::now();
//     expected_cycle_time_ = std::chrono::nanoseconds(1000000000)/frequency;
//     }
//     bool sleep(){
//     std::this_thread::sleep_until(start+ expected_cycle_time_);
//     start = std::chrono::steady_clock::now();
//     return true;
//     };
//     void reset(){
//     start = std::chrono::steady_clock::now();
//     }
// };
class controlal{
public:
    enum Mode {
        RealTime,
        Table
    };
    enum PlanType {
        X,
        Y,
        ROTATE
    };
    controlal(Mode _mode);
    ~controlal(){
        std::cout<<"fuuuuuuuuuuuuuuuccccccccccccccck"<<std::endl;
    }
    bool control_done();
    void sendCommand(double x, double y, double vx, double vy);
    double get_Velocity();
    double get_TargetVelocity();
    double get_TargetPos();
    //debug module
    void debug();

private:

    double CaseX = 0.0;//debug
    double CaseY = 0.0;    
    std::vector<float> accTable;
    std::vector<float> velTable;
    std::vector<float> posTable;
    double traj_time_x = 0;
    double traj_time_acc_x = 0;
    double traj_time_dec_x = 0;
    double traj_time_flat_x = 0;
    double traj_time_y = 0;
    double traj_time_acc_y = 0;
    double traj_time_dec_y = 0;
    double traj_time_flat_y = 0;
    bool receive_command = false;
    bool control_done_flag = true;
    //Input    
    double x1 = 0;
    double y1 = 0;//目标位置(相对当前位置)

    double v0_x = 0;
    double v0_y = 0;//当前速度

    double v1_x = 0;
    double v1_y = 0;//目标速度
    Mode control_mode;
    std::mutex Mute;
    bool start_computing= false;
    double a[2] = {0}; //ax,ay calculate result for realtime
    double v[2] = {0}; //vx,vy calculate result for realtime
    double delta_pos[2] = {0}; //x,y calculate result for realtime
    double compute_motion_1d(double x1, double v0, double v1,
                            const double a_max, const double d_max, const double v_max,
                            double &traj_time, double &traj_time_acc, double &traj_time_dec, double &traj_time_flat,
                            Mode mode,double dt,PlanType PT);
    double get_t_max(double x1, double v0, double v1, double a_max, double d_max, double v_max);
    void compute_motion_2d(double &x1, double &v0_x, double v1_x, 
                            double &y1, double &v0_y, double v1_y,
                            const double a_max, const double d_max, const double v_max,
                            double &traj_time_x, double &traj_time_acc_x, double &traj_time_dec_x, double &traj_time_flat_x,
                            double &traj_time_y, double &traj_time_acc_y, double &traj_time_dec_y, double &traj_time_flat_y,
                            Mode mode,double dt);
    void local_planner_thread_func();
    void reset();
    void compute();
};
inline double controlal::get_Velocity(){
    return v0_x,v0_y;
}
inline double controlal::get_TargetVelocity(){
    return v1_x,v1_y;
}
inline double controlal::get_TargetPos(){
    return x1,y1;
}
inline bool controlal::control_done(){
    return control_done_flag;
}
#endif
