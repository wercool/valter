#!/bin/bash

while [ 1 ]; do
    ping -c 1 192.168.0.1

    if [ $? -ne 0 ]
    then
        usb_modeswitch -R -v 04e8 -p 1234
    fi
    sleep 10
done
