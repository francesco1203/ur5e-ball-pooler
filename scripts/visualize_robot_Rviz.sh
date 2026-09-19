#!/bin/bash

#------------------------------------------------
# PARAMETRI DI SCRIPT PER LA LEGGIBILITA' E LA MANUTENIBILITA'
WS_DIR="ws_ur5e_ballpool"
INSTALL_DIR="${WS_DIR}/install"
INSTALL_SETUP_BASH="${WS_DIR}/install/setup.bash"

source ${INSTALL_SETUP_BASH}
#------------------------------------------------

real_robot="true"       #true se vuoi usare la simulazione, false se vuoi usare il robot reale



if [ "$real_robot" = "true" ]; then
    #visualizzo il robot vero da driver
    rviz2 -d $(ros2 pkg prefix arm_description)/share/arm_description/rviz/view_robot.rviz
else
    #visualizzo modello arm.urdf.xacro in RViz
    ros2 launch arm_description ur5e_description.launch.py
fi
