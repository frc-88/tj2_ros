#!/bin/bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

echo "Running set_rtc_time systemd service uninstall script"

systemctl disable set_rtc_time.service
systemctl stop set_rtc_time.service

rm /etc/systemd/system/set_rtc_time.service
rm /usr/local/bin/set_rtc_time

systemctl daemon-reload

echo "set_rtc_time systemd service uninstallation complete"
