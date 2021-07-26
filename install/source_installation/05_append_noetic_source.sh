#!/usr/bin/env bash

SOURCE_COMMAND='source ${HOME}/noetic_ws/install_isolated/setup.bash'

if ! grep -qz "$SOURCE_COMMAND" ~/.bashrc; then
    echo "Appending ros setup scripts to ~/.bashrc"
    echo "$SOURCE_COMMAND" | sudo tee -a ~/.bashrc > /dev/null
fi

echo "re-open this terminal session for bashrc to go into affect"
