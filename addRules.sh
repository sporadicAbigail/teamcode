#!/bin/bash

# Check if user is root
if [[ $EUID -ne 0 ]]; then
	echo "You must be root to use this command. (Try sudo)"
	exit 1
fi

RULES=/etc/udev/rules.d/51-android.rules

# Blank the rules file before appending
echo -n "" > $RULES

## ZTE ##
echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"19d2\", MODE=\"0666\", GROUP=\"plugdev\"" >> $RULES

## Qualcomm ##
echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"05c6\", MODE=\"0666\", GROUP=\"plugdev\"" >> $RULES

## Motorola ##
echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"22b8\", MODE=\"0666\", GROUP=\"plugdev\"" >> $RULES

# Set proper permissions
chmod a+r $RULES
