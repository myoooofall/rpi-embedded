# this is the version with uart monitor
this is the version with uart monitor <br/>
## Add the monitor.sh 
the path of the script file :monitor.sh must be the same as mine <br/>
and get the sudo permission <br/>
```bash
sudo chmod 777 monitor.sh
```
## Add the uart_monitor.service
the service file:uart_monitor.service need to be placed in /etc/systemd/system/ <br/>
```bash
sudo nano /etc/systemd/system/uart_monitor.service
```
then copy the file in raspeberry
```bash
sudo systemctl daemon-reload <br/>
sudo systemctl enable system/uart_monitor.service
```
## Edit the source file
### device.h
### device_pigpio.h $.cpp
### robotz.h $.cpp

