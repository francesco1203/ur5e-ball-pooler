#!/bin/bash

#------------------------------------------------
# DESCRIZIONE SCRIPT
# Questo script serve per avviare tutti i nodi necessari per la simulazione del robot
# e per l'esecuzione del tiro nel biliardo.
# Configurazione fissa: MuJoCo (senza RViz), Interfaccia Grafica, Esecuzione tiro abilitata,
# Game Engine Reale e Logging disabilitato.
#------------------------------------------------

# ------------------------------------------------
# PARAMETRI DI PERSONALIZZAZIONE ESECUZIONE OFF-LINE

#simulazione
open_rviz_when_using_mujoco="false"            # Forzato a false

#scena e detection
graphical_input_scene="true"                   # Sempre interfaccia grafica
start_image_view="false"                       # Non avviare image_view

#esecuzione tiro
execute_shot="true"                            # Sempre eseguire il tiro
use_real_game_engine="true"                    # Sempre game engine reale

#logging
logging_enable="false"                         # Logging disabilitato

#monitoring
topic_monitor="false"                          # Rqt_topic (impostato a false per default)
# ------------------------------------------------

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

# ================================================
# AVVIO INTERFACCIA GRAFICA PER PIAZZARE PALLINE
# ================================================
FAKE_CAMERA_CONFIG_DIR="${WS_DIR}/src/camera_perception/fake_camera/config"

if [[ "$graphical_input_scene" == "true" ]]; then
    echo "Avvio dell'interfaccia grafica per la configurazione della scena..."
    python3 ${FAKE_CAMERA_CONFIG_DIR}/graphical_config_generator.py
    echo -e "Configurazione della scena completata.\n"
fi

# ================================================
# AVVIO SIMULATORE MUJOCO + MOVEIT (Senza RViz)
# ================================================
echo "========================================"
echo "      CONFIGURAZIONE AVVIO ROS 2        "
echo "========================================"
echo -e "\nUtilizzo esclusivo di MuJoCo impostato."

scelta_mujoco_bool="true"
MOVEIT_CONFIG_DIR="${WS_DIR}/src/moveit_config/config"

echo -e "\nAggiornamento del plugin MuJoCo nel file ur5e.ros2_control.xacro..."
python3 ${MOVEIT_CONFIG_DIR}/ros2_control_hardware_auto_switch.py mujoco

MUJOCO_AUTOMATION_DIR="${WS_DIR}/src/camera_perception/fake_camera/mujoco_automation"

echo "Aggiornamento del file della scena MuJoCo con posizione delle palline..."
python3 ${MUJOCO_AUTOMATION_DIR}/autocreate_complete_scene.py --yaml_path ${FAKE_CAMERA_CONFIG_DIR}/fake_camera_config.yaml
        
echo "Avvio MuJoCo con MoveIt (Senza RViz)..."
# sleep 1

gnome-terminal --tab --title="Moveit+MuJoCo" -- bash -c \
                "source ${INSTALL_SETUP_BASH} && \
                ros2 launch moveit_config mujoco_demo.launch.py; \
                exec bash"
# sleep 3

# ================================================
# AVVIO PERCEZIONE SIMULATA
# ================================================
echo "Avvio Fake Camera..."
gnome-terminal --tab --title="Fake Camera" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 launch fake_camera fake_camera.launch.py \
                        yaml_path:=${FAKE_CAMERA_CONFIG_DIR}/fake_camera_config.yaml; \
                    exec bash"

# ================================================
# BUILDER DELLA SCENA
# ================================================
echo "Avvio Scene Builder..."
gnome-terminal --tab --title="Scene Builder" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 run scene_description scene_builder --ros-args \
                        -p use_sim_time:=true; \
                    exec bash"


# ================================================
# ESECUZIONE TIRO
# ================================================
if [[ "$execute_shot" == "true" ]]; then

    # ================================================
    # TIRO VERO E PROPRIO (TASK NODE DI SHOT PLANNING)
    # ================================================
    SHOT_CONFIG_DIR="${WS_DIR}/src/shot_execution/shot_planning/config"

    # Definisco i parametri di base
    NODE_ARGS="--ros-args \
        --params-file ${SHOT_CONFIG_DIR}/execution_params.yaml \
        --params-file ${SHOT_CONFIG_DIR}/shot_params.yaml \
        --params-file ${SHOT_CONFIG_DIR}/planning_params.yaml \
        --params-file ${SHOT_CONFIG_DIR}/using_fake_camera.yaml \
        --params-file ${SHOT_CONFIG_DIR}/moveit_fix.yaml \
        -p use_sim_time:=true"
 
    echo "Avvio Shot Planning..."
    gnome-terminal --tab --title="Shot Planning" -- bash -c " \
        source ${INSTALL_SETUP_BASH} && \
        ros2 run shot_planning task_node ${NODE_ARGS}; \
        exec bash"
    
    # ================================================
    # GAME ENGINE REALE
    # ================================================
    GAME_ENGINE_CONFIG_DIR="${WS_DIR}/src/shot_execution/game_engine/config"

    if [[ "$use_real_game_engine" == "true" ]]; then
        echo "Avvio Game Engine Reale..."
        gnome-terminal --tab --title="Game Engine Reale" -- bash -c \
                       "source ${INSTALL_SETUP_BASH} && \
                       ros2 run game_engine game_engine --ros-args \
                           --params-file ${GAME_ENGINE_CONFIG_DIR}/game_engine_params.yaml \
                           -p use_sim_time:=true; \
                       exec bash"
    fi
fi

echo "Tutti i nodi sono stati avviati!"