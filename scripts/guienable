#!/bin/bash

if [[ "$1" == 'enable' || "$1" == 'disable' ]]; then
    if [ "$1" == 'disable' ]; then
        systemctl set-default multi-user.target --force
    fi

    systemctl $1 lightdm.service --force
    systemctl $1 graphical.target --force
    systemctl $1 plymouth.service --force

    if [ "$1" == 'enable' ]; then
        if [ ! -h /etc/systemd/system/display-manager.service ]; then
            ln -s /lib/systemd/system/lightdm.service /etc/systemd/system/display-manager.service
        fi

        systemctl set-default graphical.target --force
    fi
else
    echo 'Enables or disables GUI at boot.'
    echo "Usage : $(basename) {enable | disable}"
fi
