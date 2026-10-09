#!/bin/bash


#------------------------------------------------
# PARAMETRI DI SCRIPT PER LA LEGGIBILITA' E LA MANUTENIBILITA'
WS_DIR="ws_ur5e_ballpool"
INSTALL_DIR="${WS_DIR}/install"
INSTALL_SETUP_BASH="${WS_DIR}/install/setup.bash"
#------------------------------------------------


scelta_mujoco="s"                   # 's' se vuoi usare MuJoCo, 'n' se vuoi usare MoveIt (senza MuJoCo)
logging_enable="false"              # 'true' se vuoi fare logging, 'false' se non vuoi fare logging
only_essential_logging="false"      # 'true' se vuoi fare logging solo dei dati essenziali, 'false' se vuoi fare logging di tutti i dati


 # ================================================
# TIRO VERO E PROPRIO (TASK NODE DI SHOT PLANNING)
# ================================================
SHOT_CONFIG_DIR="${WS_DIR}/src/shot_execution/shot_planning/config"

# Definisco i parametri di base (sempre presenti)
NODE_ARGS="--ros-args \
    --params-file ${SHOT_CONFIG_DIR}/execution_params.yaml \
    --params-file ${SHOT_CONFIG_DIR}/shot_params.yaml \
    --params-file ${SHOT_CONFIG_DIR}/planning_params.yaml \
    --params-file ${SHOT_CONFIG_DIR}/moveit_fix.yaml"

# Aggiungo il file di essential_logging se richiesto (il controllo funzionerà
# anche nel caso del brutal logging attivo, perché in quel caso la variabile
# only_essential_logging è stata settata a "false" automaticamente in alto)
if [[ "$logging_enable" == "true" && "$only_essential_logging" == "true" ]]; then
    echo "Essential logging attivo: aggiungo il file di parametri di essential_logging al nodo di shot planning."
    NODE_ARGS="${NODE_ARGS} --params-file ${SHOT_CONFIG_DIR}/essential_logging_params.yaml"     
fi

#eseguo il nodo di shot planning con i parametri definiti
echo "Avvio Shot Planning..."
gnome-terminal --tab --title="Shot Planning" -- bash -c \
                "source ${INSTALL_SETUP_BASH} && \
                ros2 run shot_planning task_node ${NODE_ARGS}; \
                exec bash"

# ================================================
