#include "nrf2401.h"
#include "device_pigpio.h"

using namespace std;

// #include "2401.h"
uint8_t tx_frequency = 90;	//24L01频率; Freq 6: 24; Freq 8: 90
uint8_t rx_frequency = 90;	//24L01频率; Freq 6: 24; Freq 8: 90
uint8_t bandwidth = 25;  //24L01带宽

static bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
static uint8_t address[2][6] = {{0x00,0x98,0x45,0x71,0x10}, {0x11,0xa9,0x56,0x82,0x21}};   // 0: Robot; 1: TR

void comm_2401::start() {
    devicez::set_nrf2401_en(1);
    radio_in_use->startListening();
    zos::log("nrf2401 start!\n");
    std::jthread th_comm(&comm_2401::comm_2401_test, this);
    th_comm.detach();
    // th_comm.join();
};

void comm_2401::init_2401(RF24* radio) {
    // radio->powerDown();
    // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    // radio->powerUp();
    // std::this_thread::sleep_for(std::chrono::milliseconds(5));
    // perform hardware check
    if (!radio->begin()) {
        zos::error("radio hardware is not responding!!\n");
        return; // quit now
    }
    radio->setAutoAck(false);           // EN_AA
    radio->setChannel(tx_frequency);    // RF_CH; Freq 6: 24; Freq 8: 90
    radio->setPayloadSize(MAX_SIZE);    // RX_PW_P0
    radio->setDataRate(RF24_1MBPS);     // RF_SETUP
    radio->setPALevel(RF24_PA_HIGH);    // RF24_PA_MAX is default.
    // set the TX address of the RX node into the TX pipe
    radio->openWritingPipe(address[radioNumber]);     // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio->openReadingPipe(1, address[!radioNumber]); // using pipe 1
}

void comm_2401::config_2401(RF24* radio, uint8_t* txbuf) {
    radio->stopListening();
    int check = 0;
    for ( int i=0; i<5; i++ )
        check =  check + txbuf[i];
    check = check & 0xFF;

    if ( check == txbuf[5] ) {
        // std::cout << "2401 Config" << std::endl;
        tx_frequency = txbuf[1];
        rx_frequency = txbuf[2];
        radio->setChannel(tx_frequency);
        if ( txbuf[4] == 0x01 )
            radio->setDataRate(RF24_250KBPS);
        else if ( txbuf[4] == 0x02 )
            radio->setDataRate(RF24_1MBPS);
        else if ( txbuf[4] == 0x03 )
            radio->setDataRate(RF24_2MBPS);
    }
    else {
        std::cout << "Wrong config package" << std::endl;
    }
}

void comm_2401::set_freq(int freq) {
    int freq_real;
    if ( freq >=0 && freq < 8 ) {
        freq_real = 4*freq;
    }else if (freq >=8 && freq < 13) {
        freq_real = 4*freq + 58;
    }else {
        return;
    }
    {
        std::scoped_lock lock(mutex_comm_2401);
        radio_in_use->stopListening();
        radio_in_use->setChannel(freq_real);
        radio_in_use->startListening();
    }
    // radio_RX.setChannel(freq);
}

// std::string receiver_addr = "255.255.255.255";
// std::string multicast_addr = "233.233.233.233";
// asio::ip::udp::endpoint receiver_endpoint_rx;
// asio::ip::udp::endpoint multicast_ep;

// void init_udp();
// void receive();

int comm_2401::comm_2401_test() {

    std::stop_token stoken;
    int status_count = 0;
    
    while (true)
    {
        // zos::log("comm 2401 alive :{}\n", status_count++);
        // TODO: restart?
        // if (status_count > 50000) {
        //     socket_rx.send_to("device on: 10.12.225.200", multicast_ep);
        //     status_count = 0;
        // }
        status_count++;
        // uint8_t pipe;

        // uint8_t tx_buf[5] = {0x0,0x1,0x2,0x9,0x8};
        // if(status_count > 10) {
        //     send(std::data(tx_buf));
        //     status_count = 0;
        // }
        // devicez::set_nrf2401_en(1);
        // radio_TX.startListening();
        // std::this_thread::sleep_for(std::chrono::milliseconds(15));
        if (radio_in_use->available()) {
            std::scoped_lock lock(mutex_comm_2401);
            radio_in_use->read(rxbuf, MAX_SIZE);
            receive_flag = true;
            status_count = 0;
            // zos::log("received\n");
        }else if(status_count > 50) {
            std::scoped_lock lock(mutex_comm_2401);
            devicez::set_nrf2401_en(1);
            radio_in_use->startListening();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(3));

        continue;
        
        
        // if (radio_RX.available()) {                        // is there a payload? get the pipe number that recieved it
        //     // uint8_t bytes = radio_RX.getPayloadSize();          // get the size of the payload
        //     std::scoped_lock lock(mutex_comm_2401);
        //     // zos::log("new pack\n");
        //     radio_RX.read(rxbuf, MAX_SIZE);                     // fetch payload from FIFO
        //     receive_flag = true;

        //     // std::string str(rxbuf,rxbuf+MAX_SIZE);

        //     // socket_rx.send_to(str, receiver_endpoint_rx);


        //     // char pAscii[MAX_SIZE];
        //     // HexToAscii(rxbuf, pAscii, MAX_SIZE);
        //     // std::string ascii(pAscii);
        //     status_count = 0;
        //     // cout << str << endl;
        // }else if(status_count > 50) {
        //     std::scoped_lock lock(mutex_comm_2401);
        //     change_mode_to_RX();
        //     status_count = 0;
        // }
        // std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    zos::error("2401 die!\n");
    return 0;
}

void comm_2401::HexToAscii(unsigned char * pHex, char * pAscii, int nLen)
{
    unsigned char Nibble[2];
    for (int i = 0; i < nLen; i++)
    {
        Nibble[0] = (pHex[i] & 0xF0) >> 4;
        Nibble[1] = pHex[i] & 0x0F;
        for (int j = 0; j < 2; j++)
        {
            if (Nibble[j] < 10)
                Nibble[j] += 0x30;
            else
            {
                if (Nibble[j] < 16)
                    Nibble[j] = Nibble[j] - 10 + 'a';
            }
            *pAscii++ = Nibble[j];
        }	// for (int j = ...)
    }	// for (int i = ...)
}

bool comm_2401::get_receive_flag() {
    return receive_flag;
}

void comm_2401::set_receive_flag() {
    receive_flag = false;
}

uint8_t* comm_2401::get_rxbuf() {
    std::scoped_lock lock(mutex_comm_2401);
    return rxbuf;
}

void comm_2401::change_mode() {
    if(mode == TX)  change_mode_to_RX();
    else    change_mode_to_TX();
}

void comm_2401::change_mode_to_TX() {
    // enable TX
    mode = TX;
    radio_RX.endTransaction();
    radio_TX.beginTransaction();
    radio_TX.stopListening();
    std::this_thread::sleep_for(std::chrono::microseconds(50));
    // zos::log("change mode to TX\n");
}

void comm_2401::change_mode_to_RX() {
    // enable RX
    mode = RX;
    radio_TX.endTransaction();
    radio_RX.beginTransaction();
    radio_RX.startListening();
    std::this_thread::sleep_for(std::chrono::microseconds(50));
    // gpioWrite(7, HIGH);
    // gpioWrite(8, LOW);
    // zos::log("change mode to RX\n");
}


void comm_2401::send(const void* tx_buf) {
    std::scoped_lock lock(mutex_comm_2401);
    // change_mode_to_TX();
    devicez::set_nrf2401_en(0);
    radio_in_use->stopListening();

    if (radio_in_use->write(tx_buf, MAX_SIZE)) {
        // zos::status("pack transmission success\n");
    }else {
        zos::warning("pack transmission failed\n");
    }
    devicez::set_nrf2401_en(1);
    radio_in_use->startListening();

    // change_mode();
    // radio_TX.stopListening();   // put radio_TX in TX mode
    // radio_TX.write(tx_buf, MAX_SIZE);
    // zos::log("send data :{}\n", tx_buf);

    // change_mode_to_RX();
}
