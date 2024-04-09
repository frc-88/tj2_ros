#!/bin/bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

echo "Running remember_time systemd service uninstall script"

BASE_DIR=$(realpath "$(dirname $0)")

systemctl disable remember_time.service
systemctl stop remember_time.service

rm ${BASE_DIR}/remember_time.service /etc/systemd/system/
rm ${BASE_DIR}/remember_time /usr/local/bin/

systemctl daemon-reload

echo "remember_time systemd service uninstallation complete"
