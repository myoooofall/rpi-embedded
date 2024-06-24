#ifndef SENSOR_H
#define SENSOR_H



#include <iostream>
class sensor{ 
public:
    sensor(){
        std::cout<<"sensor::sensor()"<<std::endl;
    }
    ~sensor(){
        std::cout<<"sensor::~sensor()"<<std::endl;
    }
    void monitor(){
        getVelocity();
    }
private:    
    double now_vx_from_sensor = 0;
    double now_vy_from_sensor = 0;    
    double getVelocity(){
    return now_vx_from_sensor,now_vy_from_sensor;
    }
};
#endif
