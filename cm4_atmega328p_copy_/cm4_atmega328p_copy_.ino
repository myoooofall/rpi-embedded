#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3);  // RX, TX

#define INFRA   A0
#define BATVOL  A1
#define CAPVOL  A2

#define LED7    7
#define LED8    8
#define LED9    9
#define LED10   10

bool led_heart_status = 1;
bool led_status = 0;
bool led_failed_status = 0;
bool new_string_flag = 0;

float adc_k = 3.3/1023;
float bat_k = 9.25;
float cap_k = 113.73;

int count = 0;
int count_rx_esp32 = 0;

int robot_num = -1;
int robot_freq = -1;

void setup() {
    // put your setup code here, to run once:
    //  UART_init(103);
    pinMode(LED7, OUTPUT);
    pinMode(LED8, OUTPUT);
    pinMode(LED9, OUTPUT);
    pinMode(LED10, OUTPUT);
    pinMode(4, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);  // set LED pin as output
    //  digitalWrite(LED_BUILTIN, HIGH);    // switch off LED pin

    Serial.begin(115200);
    mySerial.begin(9600);
}

void loop() {
    // put your main code here, to run repeatedly:

    //  while (Serial.available() >= 0) {
    //    char receivedData = Serial.read();
    //    if (receivedData == '1') {
    //      digitalWrite(LED_BUILTIN, HIGH);
    //    }
    //    else if (receivedData == '0') {
    //      digitalWrite(LED_BUILTIN, LOW);
    //    }
    //    delay(500);
    //  }
    //  Serial.print('1');
    //  if (Serial.available()){
    //    UART_putc('1');
    //    Serial.println('1');
    //    mySerial.println('1');
    //    char receivedData = mySerial.read();
    //    if (receivedData == '1') {
    //
    //      digitalWrite(10, HIGH);
    //      delay(100);
    //      digitalWrite(10, LOW);
    //      delay(100);
    //    }
    //    else if (receivedData == '1') {
    //  }

//    if (count_rx_esp32 > 10) {
//        digitalWrite(LED9, HIGH);
//        count_rx_esp32 = 0;
//    }else {
//        digitalWrite(LED9, LOW);
//    }


    // Start HERE!
//    if (Serial.available()) {
//        String readString = "message from nano: ";
//        while (Serial.available()) {
//            char c = Serial.read();  // gets one byte from serial buffer
//            readString += c;         // makes the string readString
//            digitalWrite(9, led_status);
//            led_status = !led_status;
//            delay(3);  // delay to allow buffer to fill
//        }
//        Serial.println(readString);
//    } else {
//        digitalWrite(10, led_failed_status);
//        led_failed_status = !led_failed_status;
//    }

    uint8_t buff[5] = {0};

    int robot_freq_temp = robot_freq;
    int robot_num_temp = robot_num;
    
    if (mySerial.available() >= 2) {
        char buff1 = mySerial.read();
        if ((buff1 & 0xf0) == 0xf0) {
            robot_freq_temp = buff1 & 0x0f;
        }
        char buff2 = mySerial.read();
        if ((buff2 & 0xf0) == 0xa0) {
            robot_num_temp = buff2 & 0x0f;
        }
        if(robot_freq_temp != robot_freq || robot_num_temp != robot_num) {
            robot_freq = robot_freq_temp;
            robot_num = robot_num_temp;
            buff[0] = 0xaa;
            buff[1] = robot_freq;
            buff[2] = robot_num;
            Serial.write(buff, 3);
            delay(4);
        }
    }

    int infrare_raw = analogRead(INFRA);
    int bat_vol_raw = analogRead(BATVOL);
    int cap_vol_raw = analogRead(CAPVOL);

    float infrare = infrare_raw * adc_k;
    float bat_vol = bat_vol_raw * adc_k * bat_k;
    float cap_vol = cap_vol_raw * adc_k * cap_k;
    if(infrare > 3) {
        digitalWrite(LED7, HIGH);
    }else {
        digitalWrite(LED7, LOW);
    }
    if(bat_vol < 15.2) {
        digitalWrite(LED8, HIGH);
    }else {
        digitalWrite(LED8, LOW);
    }
    if(cap_vol < 100) {
        digitalWrite(LED9, HIGH);
    }else {
        digitalWrite(LED9, LOW);
    }
    buff[0] = ((infrare>3) ? 1 : 0) + 0xfa;
    buff[1] = (int)cap_vol & 0xff;
    buff[2] = ((int)(bat_vol*10)) & 0xff;
    
    Serial.write(buff, 3);
    delay(4);
//    if (robot_num >=0 && robot_freq >=0) {
//        Serial.print("num: ");
//        Serial.print(robot_num);
//        Serial.print("   freq:");
//        Serial.println(robot_freq);
//    }
    

//    Serial.println("infrare: "+(String)infrare_raw);
//    Serial.println("battery voltage: "+(String)bat_vol+"      cap voltage: "+(String)cap_vol);
//    Serial.println();


    digitalWrite(LED_BUILTIN, led_heart_status);  // switch LED On
    if (count++ > 100) {
        led_heart_status = !led_heart_status;
        count = 0;
    }
    

//    digitalWrite(LED7, LOW);
//    digitalWrite(LED8, LOW);
//    digitalWrite(LED9, LOW);
//    digitalWrite(LED10, LOW);
//    if(infrare < 1) {
//      digitalWrite(LED7, HIGH);
//    }else if(infrare > 1 && infrare < 2) {
//      digitalWrite(LED8, HIGH);
//    }else if(infrare > 2 && infrare < 3) {
//      digitalWrite(LED9, HIGH); 
//    }else if(infrare > 3) {
//      digitalWrite(LED10, HIGH);
//    }
}
