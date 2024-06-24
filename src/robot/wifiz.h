#ifndef WIFI_H
#define WIFI_H

#include "zos/socket.h"
#include "zos/log.h"
#include <fmt/core.h>
#include "config.h"

extern std::atomic_bool Received_packet;
extern uint8_t rxbuf[25];
extern std::string rxbuf_proto;
extern std::mutex mutex_comm;

class ContextManager{
    std::atomic<bool> _running = false;
    void run(){
        _running = true;
        _context->run();
        _context->reset();
        _running = false;
        // zos::status("_asio_context exit\n");
    }
    std::mutex _mutex;
    std::thread _thread;
    asio::io_context* _context;
public:
    ContextManager(asio::io_context* const context):_context(context){}
    ~ContextManager(){
        stop();
        if(_thread.joinable()){
            _thread.join();
        }
    }
    void start(){
        if(_running){
            return;
        }
        std::scoped_lock<std::mutex> lock(_mutex);
        if(!_running){
            if(_thread.joinable()){
                _thread.join();
            }
            _thread = std::thread(&ContextManager::run,this);
        }
    }
    void stop(){
        std::scoped_lock<std::mutex> lock(_mutex);
        if(_running){
            _context->stop();
        }
    }
};

class wifi_comm {
public:
    // std::thread receiveThread;
    wifi_comm(const zos::udp::__callback_ep_type& f={});
    ~wifi_comm();
    void udp_stop();
    void udp_restart();
    void udp_receiver();
    void udp_sender(const void* p,const size_t size);
    void udp_sender_mc(const void* p,const size_t size);
    void set_master_ip(const std::string& ip){send2master_endpoint.address(asio::ip::address::from_string(ip)); _master_ip = ip;}
    std::string get_master_ip(){return _master_ip;}
    std::string get_ip(){ return _self_ip;}
private:
    ContextManager _context_manager;
    void _cb(const void*, size_t);
    asio::ip::udp::endpoint receiver_endpoint;
    asio::ip::udp::endpoint multicast_endpoint;
    asio::ip::udp::endpoint send2master_endpoint;

    zos::udp::socket socket_send2master;
    zos::udp::socket socket_send_multicast;
    zos::udp::socket socket_receiver;
    std::string _self_ip = "";
    std::string _master_ip = "";
};

#endif