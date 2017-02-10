#!/bin/bash

echo "Please connect the phone via USB to begin..."
echo "Once the phone has been connected, please press Enter"
read
adb tcpip 5555
echo "Please disconnect the phone from the computer."
echo "Please enter the ip address of the phone you would like to connect to: "
read IP
adb connect $IP:5555
