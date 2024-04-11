#!/bin/bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

echo "Running remember_time systemd service uninstall script"

systemctl disable remember_time.service
systemctl stop remember_time.service

rm /etc/systemd/system/remember_time.service
rm /usr/local/bin/remember_time

systemctl daemon-reload

echo "remember_time systemd service uninstallation complete"
