#!/usr/bin/env bash
if [ "$EUID" -ne 0 ]
    then echo "Please run as root"
    exit
fi

echo "Running tj2_ros systemd service uninstall script"

BASE_DIR=$(realpath "$(dirname $0)")

if [ "${BASE_INSTALL_DIR}" = "" ]; then
  BASE_INSTALL_DIR=/usr/local
fi

SERVICE_NAME=tj2_ros.service
SCRIPT_NAME=run_containers
STOP_SCRIPT_NAME=stop_tj2_ros_container

BIN_INSTALL_DIR=${BASE_INSTALL_DIR}/bin

SERVICE_ROOT_DIR=/etc/systemd/system/

rm ${SERVICE_ROOT_DIR}/${SERVICE_NAME}

rm ${BIN_INSTALL_DIR}/${SCRIPT_NAME}
rm ${BIN_INSTALL_DIR}/${STOP_SCRIPT_NAME}

echo "Disabling systemd services"
systemctl stop ${SERVICE_NAME}
systemctl disable ${SERVICE_NAME}
systemctl daemon-reload

echo "tj2_ros systemd service uninstallation complete"
