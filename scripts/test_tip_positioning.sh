#!/bin/bash


# SOLO SIMULATO PER ORA -> DA FARE PER ROBOT REALE

#------------------------------------------------
# PARAMETRI DI SCRIPT PER LA LEGGIBILITA' E LA MANUTENIBILITA'
WS_DIR="ws_ur5e_ballpool"
INSTALL_DIR="${WS_DIR}/install"
INSTALL_SETUP_BASH="${WS_DIR}/install/setup.bash"
#------------------------------------------------


# ------------------------------------------------
# Verifica preliminare della cartella di lavoro
if [ ! -d "${INSTALL_DIR}" ]; then
    echo "Errore: Cartella 'install' non trovata!"
    echo "Assicurati di aver buildato il workspace ROS 2 e di lanciare questo script dalla root del progetto."
    exit 1
fi
# ------------------------------------------------


# ------------------------------------------------
# Parametri di esecuzione
execute_test="true"  # Se impostato a "true", esegue il test
# ------------------------------------------------



# ================================================
# MOVEIT + RVIZ
# ================================================

# aggiornamento del plugin di MockHardware nel file ur5e.ros2_control.xacro
MOVEIT_CONFIG_DIR="${WS_DIR}/src/moveit_config/config"

echo "Aggiornamento del plugin MockHardware nel file ur5e.ros2_control.xacro..."
python3 ${MOVEIT_CONFIG_DIR}/ros2_control_hardware_auto_switch.py mock

echo "Avvio MoveIt con RViz..."
sleep 2

gnome-terminal --tab --title="MoveIt+Rviz" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 launch moveit_config demo.launch.py; \
                    exec bash"
sleep 10
#------------------------------------------------



# ================================================
# AVVIO PERCEZIONE SIMULATA
# ================================================

FAKE_CAMERA_CONFIG_DIR="${WS_DIR}/src/camera_perception/fake_camera/config"

echo "Avvio Fake Camera..."
gnome-terminal --tab --title="Fake Camera" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 launch fake_camera fake_camera.launch.py \
                        yaml_path:=${FAKE_CAMERA_CONFIG_DIR}/fake_camera_config.yaml; \
                    exec bash"
sleep 2



# ================================================
# BUILDER DELLA SCENA
# ================================================
echo "Avvio Scene Builder..."
gnome-terminal --tab --title="Scene Builder" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 run scene_description scene_builder; \
                    exec bash"

sleep 2



# ================================================
# ESECUZIONE
# ================================================
if [[ "$execute_test" == "true" ]]; then


    # ================================================
    # TEST
    # ================================================
    SHOT_CONFIG_DIR="${WS_DIR}/src/shot_execution/shot_planning/config"

    # Definisco i parametri di base (sempre presenti)
    NODE_ARGS="--ros-args \
        --params-file ${SHOT_CONFIG_DIR}/moveit_fix.yaml"


    #eseguo il nodo
    echo "Avvio Test..."
    gnome-terminal --tab --title="Test tip positioning" -- bash -c " \
        source ${INSTALL_SETUP_BASH} && \
        ros2 run shot_planning test_tip_positioning_node ${NODE_ARGS} ; \
        exec bash"
    sleep 5
    
fi


echo "Tutti i nodi sono stati avviati!"