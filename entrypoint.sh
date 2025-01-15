#!/bin/bash

source /ros_entrypoint.sh 

# Clear the possible lock files
rm -rf vex_brain/.vex_brain_running
rm -rf model/.model_running
rm -rf vex_log/.vex_log_running

# Compile packages
echo Start of Colcon Build
colcon build --packages-select vex_message 2>vex_message/compile.log
colcon build --packages-select vex_brain 2>vex_brain/compile.log
source install/setup.bash
#colcon build --packages-select vex_log 2>vex_log/compile.log
#source install/setup.bash
echo End of Colcon Build

# Use appropritate command base on launch mode
#case $MODE in
#    "launch")
#        vex_brain\manager.sh launch
#        model\manager.sh launch
#        vex_log\manager.sh launch
#        tail -F anything
#        ;;
#    "debug")
#        bash
#        ;;
#    *)
#        echo \"Usage: $0 {launch|debug}\"
#        exit
#        ;;
#esac

exec "$@"