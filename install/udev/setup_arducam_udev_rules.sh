#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

set -e

sudo cp "${BASE_DIR}/99-arducam.rules" "/etc/udev/rules.d/"
sudo chmod 777 "/etc/udev/rules.d/99-arducam.rules"
sudo udevadm control --reload-rules && sudo udevadm trigger
