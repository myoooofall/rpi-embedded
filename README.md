# The branch includes codes related to arduino,IMU and NFC.(大杂烩)
## ARDUINO
Version2023 use an arduino nano for detect infrare ,battery voltage,capacitance voltage.Another role is determine the vehicle number and team number of the robot by using a seperate stm-board.  
#### The code in nano can be seen in cm4_atmega328p_copy_/cm4_atmega328p_copy_.ino

## IMU
The current imu we use supports a nine-axis algorithm, but we use a six-axis algorithm to obtain information because of the magnetic field interference on the robot.  
Before using the imu for the first time, we need to calibrate the imu and change it to a six-axis algorithm.
#### The code about IMU can be found in the folder: imu_calibration
#### After the calibration, you can use the code in folder:test_imu to check whether the imu can collect the correct data

## NFC
We use PN532 as the nfc detector, the code about nfc in this repo can run in Arduino Board.(Arduino Nano doesn't work due to insufficient memory )  
folder:PN532 is an external library that requires to be loaded when running the code.  
folder:write_key includes the code to change the data on IC card,which can distinguish the car number and team number of each robot .  
#### Currently we distinguish robots by changing the sixth row on IC card.
#### Warning:Before changing the IC card information, please take care to check which row is changeable and which is permanently unchangeable in the IC card to prevent the IC card from being locked.
nfc.ino is the file  which combines the original capabilities of arduino with the capabilities of nfc recognition.

