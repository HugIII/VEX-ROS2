#!/bin/bash

function find_vex_ports() {
    # Magic Function to find the VEX USB ports
    for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
        (
            syspath="${sysdevpath%/dev}"
            devname="$(udevadm info -q name -p $syspath)"
            [[ "$devname" == "bus/"* ]] && exit
            eval "$(udevadm info -q property --export -p $syspath)"
            [[ -z "$ID_SERIAL" ]] && exit
            # devname is the name of the device in /dev
            # ID_SERIAL is the name of the device "type"/manufacturer

            # Check if the device is a VEX device
            if [[ $ID_SERIAL == *"VEX_Robotics"* ]]; then
                usbinfo="$(udevadm info -n /dev/$devname)"
                # Check if the device is the COM port
                if echo "$usbinfo" | grep -q ID_USB_INTERFACE_NUM=00; then
                    echo "VEX_COM_PORT=/dev/${devname}"
                    
                fi
                # Check if the device is the User port
                if echo "$usbinfo" | grep -q ID_USB_INTERFACE_NUM=02; then
                    echo "VEX_USER_PORT=/dev/${devname}"
                fi
            fi
        )
    done
}

# Check if argument is provided
if [ -z "${1+x}" ]; then
    echo "No argument provided"
    echo  Argument \"launch\" assumed.
    argument="launch"
else
    argument=$1
fi

# Check if ports are provided
if [ -z "${VEX_COM_PORT+x}" ] && [ -z "${VEX_USER_PORT+x}" ]; then
    # Try to find the ports automatically
    echo "No ports sets, Searching for VEX USB ports..."
    VEX_PORTS=$(find_vex_ports)
    VEX_COM_PORT_COMMAND=$(echo $VEX_PORTS | grep VEX_COM_PORT)
    eval $VEX_COM_PORT_COMMAND
    VEX_USER_PORT_COMMAND=$(echo $VEX_PORTS | grep VEX_USER_PORT)
    eval $VEX_USER_PORT_COMMAND

    if [ -n "${VEX_COM_PORT+x}" ] && [ -n "${VEX_USER_PORT+x}" ]; then
        echo "VEX USB ports found."
    else
        if [ -z "${VEX_COM_PORT+x}" ]; then
            echo "No VEX COM port found."
        fi
        if [ -z "${VEX_USER_PORT+x}" ]; then
            echo "No VEX User port found."
        fi
    fi
else
    if [ -n "${VEX_COM_PORT+x}" ] || [ -n "${VEX_COM_PORT+x}" ]; then
        if [ -n "${VEX_COM_PORT+x}" ]; then
            echo "Only VEX COM port given."
        fi
        if [ -n "${VEX_USER_PORT+x}" ]; then
            echo "Only VEX User port given."
        fi
        echo "Partial setting of usb ports not supported."
    else
        echo "VEX USB ports preset."
    fi
fi

# Export variables for docker-compose file
if [ -z "${VEX_COM_PORT+x}" ] && [ -z "${VEX_USER_PORT+x}" ]; then
    # No ports found/given
    echo "No ports: defaulting to debug mode."
    export VEX_CONNECTED="false"
    argument="debug"
else
    # Ports found/given
    echo "Communication port: $VEX_COM_PORT | User port: $VEX_USER_PORT"
    export VEX_COM_PORT
    export VEX_USER_PORT
    export VEX_CONNECTED="true"
fi

# Perform the right commands based on the argument
case $argument in
    # Launch the container and detach it
    "launch" )
        export MODE="launch"
        docker compose up -d
        ;;

    # Launch container with run to debug
    "debug")
        export MODE="debug"
        docker compose run --rm --service-ports ros2_to_vex_container bash
        ;;

    # Invalid argument
    * )
        echo $1 is not a valid argument. 
        echo usage: $0 "[launch|debug]"
        exit 1
        ;;
esac

