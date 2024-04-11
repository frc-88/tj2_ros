#!/bin/bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

echo "Running set_rtc_time systemd service install script"

BASE_DIR=$(realpath "$(dirname $0)")

cp ${BASE_DIR}/set_rtc_time.service /etc/systemd/system/
cp ${BASE_DIR}/set_rtc_time /usr/local/bin/

echo "Enabling systemd services"
systemctl daemon-reload
loginctl enable-linger $USER
systemctl enable set_rtc_time.service
systemctl restart set_rtc_time.service

echo "set_rtc_time systemd service installation complete"
