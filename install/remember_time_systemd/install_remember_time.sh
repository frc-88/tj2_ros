#!/bin/bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

echo "Running remember_time systemd service install script"

BASE_DIR=$(realpath "$(dirname $0)")

cp ${BASE_DIR}/remember_time.service /etc/systemd/system/
cp ${BASE_DIR}/remember_time /usr/local/bin/

echo "Enabling systemd services"
systemctl daemon-reload
loginctl enable-linger $USER
systemctl enable remember_time.service
systemctl restart remember_time.service

echo "remember_time systemd service installation complete"
