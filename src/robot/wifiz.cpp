#include "wifiz.h"

// void wifi_comm::_cb(const void* p,size_t lens) {
//     std::scoped_lock lock(mutex_comm);
//     std::string s(static_cast<const char*>(p),lens);
//     // 更新最新数据包
//     Received_packet = true;
//     std::copy(begin(s), end(s), std::begin(rxbuf));
//     rxbuf_proto.assign(static_cast<const char*>(p), lens);
// }

wifi_comm::wifi_comm(const zos::udp::__callback_ep_type& f):_context_manager(zos::__io::_()) {
    send2master_endpoint = asio::ip::udp::endpoint(asio::ip::address::from_string("127.0.0.1"), config::send_single_port);
    multicast_endpoint = asio::ip::udp::endpoint(asio::ip::address::from_string(config::multicast_addr), config::send_multicast_port);
    receiver_endpoint = asio::ip::udp::endpoint(zos::udp::address_v4::any(), config::receive_port);
    if(f){
        auto res = socket_receiver.bind(receiver_endpoint,f);
        zos::info("wifi_comm socket bind - res : {}\n",res);
        if(res){
            udp_restart();
        }
    }else{
        zos::error("wifi_comm no callback function\n");
    }
    auto ips = zos::getNetworkInterfacesIP();
    zos::info("list all network interfaces({}):\n",ips.size());
    for(auto ip:ips){
        zos::info("\t{}\n",ip);
        if(ip.rfind(config::multicast_if_prefix,0) == 0){
            socket_send_multicast.set_interface(asio::ip::address::from_string(ip));
            _self_ip = ip;
            zos::status("choose interface : {}\n",ip);
            break;
        }
    }
}
wifi_comm::~wifi_comm(){
    _context_manager.stop();
}

void wifi_comm::udp_stop() {
    _context_manager.stop();
}

void wifi_comm::udp_restart() {
    _context_manager.start();
}

// // Receiver
// void wifi_comm::udp_receiver() {
//     asio::ip::udp::endpoint listen_ep(asio::ip::address::from_string("0.0.0.0"), config::receive_port);
//     zos::udp::socket socket;
//     socket.bind(listen_ep,std::bind(&wifi_comm::_cb,this,std::placeholders::_1,std::placeholders::_2));
//     zos::__io::_()->run();
// }

void wifi_comm::udp_sender(const void* p,const size_t size) {
    if(p == nullptr){
        zos::log("in udp_sender_normal function\n");
        socket_send2master.send_to(fmt::format("normal udp test!!!"),send2master_endpoint);
    }else{
        socket_send2master.send_to(p,size, send2master_endpoint);
    }
}

void wifi_comm::udp_sender_mc(const void* p,const size_t size) {
    if(p == nullptr){
        zos::log("in udp_sender_mc function\n");
        socket_send_multicast.send_to(fmt::format("multicast test!!!"),multicast_endpoint);
    }else{
        socket_send_multicast.send_to(p,size, multicast_endpoint);
    }
}