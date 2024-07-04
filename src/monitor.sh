#!/bin/bash
restart_flag="/home/pi/rpi-embedded/src/restart_flag.txt"
robot_service_name="zjunlict_wifi.service"

while true; do
    if [ -f "$restart_flag" ]; then
        echo "Restart flag file found. stop $robot_service_name..."
        sudo systemctl stop $robot_service_name
        sleep 0.2
        rm -f "$restart_flag"  # 删除标志文件
        echo "start $robot_service_name..."
        sudo systemctl start $robot_service_name
        echo "done restart"
    fi
        echo "uart is safe"
    sleep 50  # 每5秒检查一次
done