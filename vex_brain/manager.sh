#!/bin/bash
# This script is used to launch and shutdown the vex_brain node

# Function to kill a process and all of its children
function kill_recurse() {
    cpids=`pgrep -P $1|xargs`
    for cpid in $cpids;
    do
        kill_recurse $cpid
    done
    kill -9 $1
}

# Set the working directory to the directory of this script
run_file_name=.vex_brain_running
log_file_name=vex_brain_run.log
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
            echo "Vex_brain is already running"
            exit 1
        else
            rm -rf $log_file_name
            ros2 launch ./launch/vex_launch.launch.py >$log_file_name & 
            BRAIN_PID=$!
            echo $BRAIN_PID > $run_file_name
            echo "Vex_brain launched"
        fi
        ;;

    # Shutdown the vex_brain node
    "shutdown")
        if [ -f "$run_file_name" ]; then
            BRAIN_PID=$(cat $run_file_name)
            rm $run_file_name
            kill_recurse $BRAIN_PID
            echo "Vex_brain shutdown"
        else
            echo "Vex_brain was not running"
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