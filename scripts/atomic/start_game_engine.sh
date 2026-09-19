#!/bin/bash


#------------------------------------------------
# PARAMETRI DI SCRIPT PER LA LEGGIBILITA' E LA MANUTENIBILITA'
WS_DIR="ws_ur5e_ballpool"
INSTALL_DIR="${WS_DIR}/install"
INSTALL_SETUP_BASH="${WS_DIR}/install/setup.bash"
#------------------------------------------------


use_real_game_engine="false"                    #true se vuoi usare il game engine reale, false se vuoi usare quello fake


# ================================================
# GAME ENGINE (REALE O SIMULATO)
# ================================================
GAME_ENGINE_CONFIG_DIR="${WS_DIR}/src/shot_execution/game_engine/config"

if [[ "$use_real_game_engine" == "true" ]]; then
    
    echo "Avvio Game Engine Reale..."
    gnome-terminal --tab --title="Game Engine Reale" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 run game_engine game_engine --ros-args \
                        --params-file ${GAME_ENGINE_CONFIG_DIR}/game_engine_params.yaml; \
                    exec bash"
else
    echo "Avvio Fake Game Engine..."
    gnome-terminal --tab --title="Fake Game Engine" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 run game_engine fake_game_engine --ros-args \
                        --params-file ${GAME_ENGINE_CONFIG_DIR}/game_engine_params.yaml; \
                    exec bash"
    
fi