#ifndef NRF2401_H
#define NRF2401_H

#define MAX_SIZE 25           // this is the maximum for this example. (minimum is 1)

#include <ctime>       // time()
#include <iostream>    // cin, cout, endl
#include <string>      // string, getline()
#include <chrono>
#include <thread>
#include <time.h>      // CLOCK_MONOTONIC_RAW, timespec, clock_gettime()
#include "zos/socket.h"
#include "zos/log.h"
#include <fmt/core.h>
#include <RF24/RF24.h> // RF24, RF24_PA_LOW, delay()
#include "config.h"

// extern uint16_t Received_packet;
// extern uint8_t rxbuf[25];
// extern std::mutex mutex_comm;

/****************** Linux ***********************/
// Radio CE Pin, CSN Pin, SPI Speed
// CE Pin uses GPIO number with BCM and SPIDEV drivers, other platforms use their own pin numbering
// CS Pin addresses the SPI bus number at /dev/spidev<a>.<b>
// ie: RF24 radio(<ce_pin>, <a>*10+<b>); spidev1.0 is 10, spidev1.1 is 11 etc..
// RF24 radio_TX(22, 1);
// RF24 radio_RX(27, 0);

class comm_2401 {
public:
    comm_2401() : radio_TX(config::radio_tx_ce_pin, config::radio_tx_csn) {
        radio_in_use = &radio_TX;
        // radio_TX.startListening();
        // init_2401(&radio_RX);
        // radio_RX.startListening();
        // put radio_TX in TX mode
        // Start 2401 for receiving
        // radio_RX.powerUp();
        // init_2401(&radio_RX);
        init_2401(radio_in_use);
        start();
    };
    void start();
    void send(const void* tx_buf);
    bool get_receive_flag();
    void set_receive_flag();
    uint8_t* get_rxbuf();

    void set_freq(int freq);

    std::mutex mutex_comm_2401;
    ~comm_2401() {
        zos::log("stop 2401\n");
        gpioTerminate();
    }

private:
    RF24 radio_TX;
    RF24 radio_RX;
    RF24* radio_in_use = NULL;
    void init_2401(RF24* radio);
    void config_2401(RF24* radio, uint8_t* txbuf);
    int comm_2401_test();

    bool receive_flag = false;
    uint8_t rxbuf[MAX_SIZE] = {0x0};

    enum modes{TX = 0, RX = 1}   mode=RX;
    void change_mode();
    void change_mode_to_TX();
    void change_mode_to_RX();
    static void HexToAscii(unsigned char * pHex, char * pAscii, int nLen);
};

#endif