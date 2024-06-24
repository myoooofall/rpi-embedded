#include "controlal.h"

using namespace std;
using namespace config;
using namespace zos;

controlal::controlal(Mode _mode):control_mode(_mode){

    std::thread controlThread(&controlal::local_planner_thread_func, this);
    controlThread.detach();

}

void controlal::local_planner_thread_func(){ 

    Rate rate(control_rate);

    while(1){

        if(receive_command){
            reset();    
        }
        compute();
        rate.sleep();
    }

}

//非零速到点,实时规划，只需要返回加速度a_max/-a_max/0/d_max/-d_max,速度通过其他函数积分
double controlal::compute_motion_1d(double x1, double v0, double v1,
                                    const double a_max, const double d_max, const double v_max,
                                    double &traj_time, double &traj_time_acc, double &traj_time_dec, double &traj_time_flat,
                                    Mode mode,double dt,PlanType PT)
{

    if(mode == RealTime){            

        int dis_flag = 1;
        int vel_flag = 1;
        if(x1<0) dis_flag = -1;
        if(v1-v0<0) vel_flag = -1;
        //case1:已到点，迅速停下或已达到目标速度
        double a = dt;
        if(fabs(x1) <= v_max*dt) {
            if(fabs(v0) <= 0.5*a_max*dt) v0 = 0;
            if(PT == X)CaseX =1;
            if(PT == Y)CaseY =1;
            return (v0 ==0 )?0:vel_flag*d_max;
        }
        //case2:速度与位移同向，讨论
        if(x1*v0 >= 0 ){

            double distance = fabs(x1);
            v0 = fabs((v0>v_max)?v_max:v0);
            double to_target_min_dis = fabs(((v1*v1)-(v0*v0))/(2*a_max)) + 0.5*v0*dt;
            //if(PT==Y) cout<<v0*dt<<endl;
            double min_acc_dis = fabs(((v_max*v_max)-(v0*v0))/(2*a_max));
            double min_dec_dis = fabs(((v_max*v_max)-(v1*v1))/(2*a_max)) + 0.5*v0*dt;
            //距离够匀速
            if(min_acc_dis + min_dec_dis < distance ){
            if(PT == X)CaseX = 2.1;
            if(PT == Y)CaseY = 2.1;
            if(v0 == v_max) return 0;
            else return a_max*dis_flag;
            } 

            //距离不够，直接根据末速度加减速
            if(to_target_min_dis >= distance){
            if(PT == X)CaseX = 2.2;
            if(PT == Y)CaseY = 2.2;
            //std::cout<< PT <<endl;
            std::cout<<" to_target_min_dis: "<< to_target_min_dis<<std::endl;
            // std::cout<<" distance: "<< distance <<std::endl;
            // std::cout<<" can not achieve the position and velocity at the same time "<<std::endl;
            return a_max*vel_flag;
            }
            //距离够了,但不够匀速，根据位移加减速
            if(min_acc_dis + min_dec_dis >= distance && v0 != v_max){

            if(PT == X)CaseX = 2.3;
            if(PT == Y)CaseY = 2.3;
            return a_max*dis_flag;
            }
        }
            //case3:速度与位移反向，先减速到0，然后进入同向讨论
            if((x1) * v0 < 0 ){
                if(PT == X)CaseX = 3;
                if(PT == Y)CaseY = 3;
                int vel_flag = -1;
                if(v0<0) vel_flag = 1;
                return d_max*vel_flag;
            }
    }

    else if (mode == Table)
    {
        velTable.push_back(0);
        posTable.push_back(x1);
        accTable.push_back(0);
        double acc1_time_to_v1 = fabs(v1-v0)/a_max;
        double acc1_dist_to_v1 = fabs((v1+v0)/2.0) *acc1_time_to_v1;
        double dec1_time_to_v1 = fabs(v0-v1)/d_max;
        double dec1_dist_to_v1 = fabs((v1+v0)/2.0) *dec1_time_to_v1;
        double v_c = 0; //
        double t_max = 0; //v_max lasting time
        // (v_max+v0)(v_max-v0)/(2*a_max)+(v_max+v1)(v_max-v1)/(2*d_max)+v_max*t_max = x1
        double timecount = 0;
        if(v0*x1<0){        
            timecount = 0;
            int vel_flag = v0<0?-1:1;
            int pos_flag = x1<0?-1:1;
            v0 = fabs(v0);
            x1 = fabs(x1);
            double  t_d = v0/d_max;
            double  x_d = (v0*v0)/(2*d_max);

            while((v0-timecount*dt*d_max) >= 0){
                accTable.push_back(-1*vel_flag*d_max);
                velTable.push_back(vel_flag*(v0-a_max*dt*timecount));
                posTable.push_back(pos_flag*(x1+(pow(v0,2)-pow((v0-a_max*dt*timecount),2))/(2*d_max)));
                timecount++;
            }
            compute_motion_1d(posTable.back(), velTable.back(), v1, a_max, d_max, v_max,
            traj_time, traj_time_acc,traj_time_dec,traj_time_flat,Table,dt,PT);

        }
        else{
            timecount = 0;
            int vel_flag = v0<0?-1:1;
            int pos_flag = x1<0?-1:1;
            if(v0 == 0) vel_flag = pos_flag;
            v0 = fabs(v0);
            x1 = fabs(x1);
            t_max = get_t_max(x1,v0,v1,a_max,d_max,v_max);
            cout<<t_max<<endl;
            if(t_max >= 0){

                traj_time_acc = (v_max-v0)/a_max;
                traj_time_dec = (v_max-v1)/d_max;
                traj_time_flat = t_max;
                traj_time = traj_time_dec + traj_time_acc + traj_time_flat;
                while(timecount*dt<=traj_time_acc){
                    accTable.push_back(vel_flag*a_max);
                    velTable.push_back(vel_flag*(v0+a_max*dt*timecount));
                    posTable.push_back(posTable.back()-velTable.back()*dt);
                    timecount++;
                }
                //traj_time_acc = timecount*dt; // 加速时间更换为实际执行时间
                while(timecount*dt<=traj_time_acc+traj_time_flat){
                    accTable.push_back(0);
                    velTable.push_back(velTable.back());
                    posTable.push_back(posTable.back()-velTable.back()*dt);
                    timecount++;
                }
                //traj_time_flat = timecount*dt - traj_time_acc;//
                while(timecount*dt<=traj_time){
                    accTable.push_back(-1*d_max*vel_flag);
                    velTable.push_back(vel_flag*(v_max-d_max*(timecount*dt-traj_time_acc-traj_time_flat)));
                    posTable.push_back(posTable.back()-velTable.back()*dt);
                    timecount++;
                }


            }

            if(t_max < 0){

                if(v0>fabs(v1) && dec1_dist_to_v1>fabs(x1)){

                    traj_time_acc = 0;
                    traj_time_dec = (v0-fabs(v1) )/d_max;
                    traj_time_flat =0;
                    traj_time = traj_time_dec + traj_time_acc + traj_time_flat;
                    while(dt*timecount<=traj_time_dec){
                        accTable.push_back(-vel_flag*d_max);
                        velTable.push_back(vel_flag*(v0 - timecount*dt*d_max));
                        posTable.push_back(posTable.back()-velTable.back()*dt);
                        timecount++;
                    }

                }
                else if(v0<fabs(v1) && acc1_dist_to_v1>fabs(x1)){

                    traj_time_dec = 0;
                    traj_time_acc = (fabs(v1) - v0)/a_max;
                    traj_time_flat =0;
                    traj_time = traj_time_dec + traj_time_acc + traj_time_flat;
                    while(timecount*dt<=traj_time_acc){
                        accTable.push_back(vel_flag*a_max);
                        velTable.push_back(vel_flag*(v0 + timecount*dt*a_max));
                        posTable.push_back(posTable.back()-velTable.back()*dt);
                        timecount++;
                    }
                }
                else{
                    double v_top = 0;
                    //(pow(v_top,2)-pow(v0,2))/(2*a_max)+(pow(v_top,2)-pow(v1,2))/(2*d_max) = x1;
                    v_top = sqrt((2*a_max*d_max*x1+a_max*v1*v1+d_max*v0*v0)/(a_max+d_max));
                    traj_time_dec = (v_top - fabs(v1))/d_max;
                    traj_time_acc = (v_top - v0)/a_max;
                    traj_time_flat =0;
                    traj_time = traj_time_dec + traj_time_acc + traj_time_flat;
                    while(timecount*dt<=traj_time_acc){
                        accTable.push_back(vel_flag*a_max);
                        velTable.push_back(vel_flag*(v0+a_max*dt*timecount));
                        posTable.push_back(posTable.back()-velTable.back()*dt);
                        timecount++;
                    }

                    while(timecount*dt<=traj_time_acc+traj_time_dec){
                        accTable.push_back(-vel_flag*d_max);
                        velTable.push_back(vel_flag*(v0-d_max*dt*timecount));
                        posTable.push_back(posTable.back()-velTable.back()*dt);
                        timecount++;
                    }
                }
            }
        }
    }
    return 0;
}


double controlal::get_t_max(double x1, double v0, double v1, 
                            double a_max, double d_max, double v_max){

    double x_max = x1-(pow(v_max,2)-pow(v0,2))/(2*a_max)-(pow(v_max,2)-pow(v0,2))/(2*d_max);
    double t = x_max/v_max;
    return t;

}


void controlal::compute_motion_2d(double &x1, double &v0_x, double v1_x, 
                                  double &y1, double &v0_y, double v1_y,
                                const double a_max, const double d_max, const double v_max,
                                double &traj_time_x, double &traj_time_acc_x, double &traj_time_dec_x, double &traj_time_flat_x,
                                double &traj_time_y, double &traj_time_acc_y, double &traj_time_dec_y, double &traj_time_flat_y,
                                Mode mode,double dt){

    if(mode == RealTime){

        a[0] = compute_motion_1d(x1, v0_x, v1_x, a_max, d_max, v_max,
        traj_time_x, traj_time_acc_x,traj_time_dec_x,traj_time_flat_x,
        mode,dt,X);

        a[1] = compute_motion_1d(y1, v0_y, v1_y, a_max, d_max, v_max,
        traj_time_y, traj_time_acc_y,traj_time_dec_y,traj_time_flat_y,
        mode,dt,Y);
        //new velocity and send to function motionplanner and for next time compute_motion_2d
        v[0] = v0_x + a[0]*dt;
        v[1] = v0_y + a[1]*dt;

        if(v[0]<-v_max) v[0] = -v_max;
        if(v[0]> v_max) v[0] =  v_max;
        if(v[1]<-v_max) v[1] = -v_max;
        if(v[1]> v_max) v[1] =  v_max;
        if(fabs(v[0])<0.5*a_max*dt) v[0] = 0;
        if(fabs(v[1])<0.5*a_max*dt) v[1] = 0;
        //new position for next time compute_motion_2d
        delta_pos[0] = (v[0]+v0_x)*dt/2;
        delta_pos[1] = (v[1]+v0_y)*dt/2; 

        v0_x = v[0];
        v0_y = v[1];

        x1 = x1 - delta_pos[0];
        y1 = y1 - delta_pos[1];
    }

    else if(mode == Table){

        compute_motion_1d(x1, v0_x, v1_x, a_max, d_max, v_max,
        traj_time_x, traj_time_acc_x,traj_time_dec_x,traj_time_flat_x,
        mode,dt,X);

        compute_motion_1d(y1, v0_y, v1_y, a_max, d_max, v_max,
        traj_time_y, traj_time_acc_y,traj_time_dec_y,traj_time_flat_y,
        mode,dt,Y);

    }
}
void controlal::sendCommand(double x, double y, double vx, double vy){

    x1 = x;
    y1 = y;
    v1_x = vx;
    v1_y = vy;
    receive_command = true;

}
void controlal::reset(){

    sensor Sensor;
    Sensor.monitor();
    receive_command = false;

}

void controlal::compute(){

    if(control_mode == RealTime){

        if(fabs(x1)>v0_x*dt | fabs(y1)>v_max*dt | fabs(v0_x)>a_max*dt | fabs(v0_y)>a_max*dt){

            Mute.lock();
            start_computing = true;
            control_done_flag = false;
            compute_motion_2d(x1,v0_x,v1_x,y1,v0_y,v1_y,a_max,d_max,v_max,
            traj_time_x,traj_time_acc_x,traj_time_dec_x,traj_time_flat_x,traj_time_y,traj_time_acc_y,
            traj_time_dec_y,traj_time_flat_y,control_mode,dt);
            Mute.unlock();

        } 
        else{

            if(!start_computing && !control_done_flag){
            cout<<"no command"<<endl;
            }
            if(start_computing){
            control_done_flag = true;
            start_computing = false;
            cout<<"finished"<<endl;
            }
        }
    }

    else if(control_mode == Table){

        compute_motion_2d(x1,v0_x,v1_x,y1,v0_y,v1_y,a_max,d_max,v_max,
        traj_time_x,traj_time_acc_x,traj_time_dec_x,traj_time_flat_x,traj_time_y,traj_time_acc_y,
        traj_time_dec_y,traj_time_flat_y,control_mode,dt);


    }

}

void controlal::debug(){

    if(CONTROL_DEBUGGER){

        cout << "accTabel:" << endl;
        for (int i = 0; i < accTable.size(); i++ )
        {
            cout << accTable[i] << " ";
        }

        cout<<endl;
        cout << endl << "velTable:" << endl;
        for ( int i = 0; i < velTable.size(); i++ )
        {
            cout << velTable[i] << " ";
        }
            cout <<endl << "posTable:" << endl;
        for ( int i = 0; i < posTable.size(); i++ )
        {
            cout << posTable[i] << " ";
        }
        cout << endl;

    }

}
