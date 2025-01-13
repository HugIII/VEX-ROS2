#!/bin/bash
# This script is used to launch and shutdown the vex_brain node

# Set the working directory to the directory of this script
run_file_name=.model_running
log_file_name=model_run.log
cd ${0%/*}

# Check if an argument was provided
if [ -z "${1+x}" ]; then
    echo "No argument provided"
    echo usage: $0 "[launch|shutdown]"
    exit 1
fi

# Check the argument and perform the appropriate action
case $1 in
    # Launch the vex_brain node
    "launch" )
        if [ -f "$run_file_name" ]; then
            echo "model is already running"
            exit 1
        else
            rm -rf $log_file_name
            ros2 run model model >$log_file_name & 
            BRAIN_PID=$!
            echo $BRAIN_PID > $run_file_name
            echo "model launched"
        fi
        ;;

    # Shutdown the vex_brain node
    "shutdown")
        if [ -f "$run_file_name" ]; then
            BRAIN_PID=$(cat $run_file_name)
            rm $run_file_name
            ros2 topic pub -1 /terminate_sim std_msgs/msg/Empty> /dev/null
            echo "model shutdown (id: $BRAIN_PID)"
        else
            echo "model was not running"
            exit 1
        fi
        ;;

    # Invalid argument
    * )
        echo $1 is not a valid argument. 
        echo usage: $0 "[launch|shutdown]"
        exit 1
        ;;
esac