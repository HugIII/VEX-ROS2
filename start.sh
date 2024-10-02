#!/bin/bash

if [ -z "${1+x}" ]; then
    echo "No argument provided"
    echo  Argument \"launch\" assumed.
    docker compose up -d
fi

case $1 in
    # Launch the container and detach it
    "launch" )
        export MODE="launch"
        docker compose up -d
        ;;

    # Launch container with run to debug
    "debug")
        export MODE="debug"
        docker compose run --rm --service-ports ros2_galactic
        ;;

    # Invalid argument
    * )
        echo $1 is not a valid argument. 
        echo usage: $0 "[launch|debug]"
        exit 1
        ;;
esac

