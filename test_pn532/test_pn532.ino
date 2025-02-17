#include <Wire.h>
#include <Adafruit_PN532.h>
 
#define SDA_PIN A4
#define SCL_PIN A5
 
Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);
 
void setup(void) {
    Serial.begin(115200);
 
    nfc.begin();
 
    uint32_t versiondata = nfc.getFirmwareVersion();
    if (!versiondata) {
        Serial.print("Didn't find PN53x board");
        while (1);
    }
 
    nfc.SAMConfig();
    Serial.println("Waiting for NFC card...");
}
 
void loop(void) {
    uint8_t success;
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
    uint8_t uidLength;
 
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
 
    if (success) {
        Serial.println("Found an NFC card!");
        Serial.print("UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
        Serial.print("UID Value: ");
        for (uint8_t i=0; i < uidLength; i++) {
            Serial.print(" 0x");Serial.print(uid[i], HEX);
        }
        Serial.println("");
        

    
        delay(1000);
    }
}
