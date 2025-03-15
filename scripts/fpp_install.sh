#!/bin/bash
echo "Running install script for FPP-USB-Relay-Ctrl"
CORE_SO="/usr/lib/libfpp-co-USBRelay.so"
if [ ! -f "$CORE_SO" ]; then
    echo "Installing libfpp-co-USBRelay.so..."
    cd /opt/fpp/src/
    make libfpp-co-USBRelay.so
    if [ -f /opt/fpp/src/libfpp-co-USBRelay.so ]; then
        sudo cp /opt/fpp/src/libfpp-co-USBRelay.so /usr/lib/
        sudo chown fpp:fpp /usr/lib/libfpp-co-USBRelay.so
        sudo chmod 644 /usr/lib/libfpp-co-USBRelay.so
        echo "Installed libfpp-co-USBRelay.so to /usr/lib/"
    else
        echo "Warning: Failed to build libfpp-co-USBRelay.so."
    fi
else
    echo "libfpp-co-USBRelay.so already present in /usr/lib/."
fi
echo "Install complete"
echo "Note: Restart fppd to apply plugin updates (sudo systemctl restart fppd)"