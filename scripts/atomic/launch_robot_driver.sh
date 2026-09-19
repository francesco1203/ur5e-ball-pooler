#!/bin/bash


#------------------------------------------------
# PARAMETRI DI SCRIPT PER LA LEGGIBILITA' E LA MANUTENIBILITA'
WS_DIR="ws_ur5e_ballpool"
INSTALL_DIR="${WS_DIR}/install"
INSTALL_SETUP_BASH="${WS_DIR}/install/setup.bash"

source "${INSTALL_SETUP_BASH}"
#------------------------------------------------

# ------------------------------------------------
# PARAMETRI DI RETE
LAB_ROBOT_IP="192.168.1.110"

URSIM_ROBOT_IP="192.168.56.101"
UR_SIM_REVERSE_IP="192.168.56.1"  #il tuo ip sulla rete del robot simulato
#------------------------------------------------


ARM_DESC_DIR="$(ros2 pkg prefix arm_description)/share/arm_description"


if [[ "$scelta_URSim" =~ ^[sS][iI]?$ ]]; then

    #robot simulato
    gnome-terminal --tab --title="Driver UR5e" -- bash -c "source \"${INSTALL_SETUP_BASH}\" && \
        ros2 launch ur_robot_driver ur_control.launch.py \
        ur_type:=ur5e \
        robot_ip:=${URSIM_ROBOT_IP} \
        reverse_ip:=${UR_SIM_REVERSE_IP} \
        launch_rviz:=false \
        description_file:=${ARM_DESC_DIR}/urdf/arm_driver_wrapper.urdf.xacro; \
        exec bash"
else

    #robot reale
    gnome-terminal --tab --title="Driver UR5e" -- bash -c "source \"${INSTALL_SETUP_BASH}\" && \
        ros2 launch ur_robot_driver ur_control.launch.py \
        ur_type:=ur5e \
        robot_ip:=${LAB_ROBOT_IP} \
        launch_rviz:=false \
        description_file:=${ARM_DESC_DIR}/urdf/arm_driver_wrapper.urdf.xacro \
        kinematics_params_file:=${ARM_DESC_DIR}/config/uclv_right_ur5e_kinematics.yaml; \
        exec bash"
fi