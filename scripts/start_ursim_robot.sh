#!/bin/bash


#------------------------------------------------
# DESCRIZIONE SCRIPT

# Questo script serve per avviare tutti i nodi necessari per la simulazione del robot in UrSim
# al fine di testare i driver del robot e familiarizzare con il teach pendant e PolYScope.
# Per muovere il robot simulato, dopo aver seguito la corretta sequenza di avvio, 
# eseguiremo il tiro con terne ideali (fake camera) e lo visualizzeremo in RViz.
#------------------------------------------------



# ------------------------------------------------
# PARAMETRI DI RETE
URSIM_ROBOT_IP="192.168.56.101"   #default ip del robot simulato (URSim) sulla rete virtuale creata da VirtualBox
UR_SIM_REVERSE_IP="192.168.56.1"  #il tuo ip sulla rete del robot simulato

robot_ip="${URSIM_ROBOT_IP}"
reverse_ip="${UR_SIM_REVERSE_IP}"
#------------------------------------------------


# ------------------------------------------------
# PARAMETRI DI PERSONALIZZAZIONE ESECUZIONE OFF-LINE

#launch container (solo per simulazione con URSim, non serve per robot reale)
launch_container="true"                     

#launch driver (se robot/container è già avviato e il driver è stato già lanciato, non serve rilanciare tutto)
launch_robot_driver="true"                         

#avvio camera IntelRealsense
use_real_camera="true"        #se false, uso le terne ideali di fake camera
start_image_view="true"       #simulatore per la percezione             

#esecuzione tiro
execute_shot="true"                             

#costruzione scena (biliardino - aggiornamento della grafica delle palline in scena)  
oneshot_build_scene="false"                     #metti a true per costuire una sola volta la scena               
auto_loop_build_scene="true"                    #metti a true per aggiornare la scena in loop (utile per vedere le palline muoversi)
time_between_scene_builds=1                     
user_input_to_build_scene="false"               #metti a true per aggiornare la scena iterativamente, ma solo quando premi [INVIO] (utile per debug e test)

                      
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
    echo "Assicurati di lanciare questo script dalla root del workspace ROS 2."
    exit 1
fi
source "${INSTALL_SETUP_BASH}"
# ------------------------------------------------



# ================================================
# ADDESTRAMENTO DELL'UTENTE
# ================================================
echo ""
echo "========================================"
echo "      CONFIGURAZIONE AVVIO ROS 2        "
echo "========================================"

    cat << 'EOF'

Sequenza corretta in simulazione UrSim (UR5e):

1. Avvio Container Docker (URSim) ✅
    - Lo script lancia il container
    - Attendi che si apra l'interfaccia URSim (all'ip 192.168.56.101)
2. Accensione Robot Virtuale ✅
    - Nel teach pendant di URSim: clicca "Power Off" → "Normal"
    - Attendi che il robot si accenda (led diventa verde)
    - Lo script aspetta che tu prema un tasto per confermare
3. Avvio Driver ✅
    - Il driver parte e si connette al robot
    - MA il robot NON è ancora pronto a ricevere comandi!
4. Avvio External Control Program ⚠️ PASSAGGIO CRUCIALE
    - Nel teach pendant di URSim:
        - Vai su "Program" → "URCaps" → "External Control"
    - Verifica che l'IP sia corretto (192.168.56.1)
    - Premi "Play" ▶️
    - Solo ora il robot accetta comandi dal driver
    - Lo script aspetta che tu prema un tasto dopo aver visto "Robot ready to receive commands"
5. Avvio MoveIt ✅
    - Ora MoveIt può comandare il robot

Quindi: Docker → Accendi Robot → Driver → External Control → MoveIt 🎯
EOF

sleep 1


# ================================================
# AGGIORNAMENTO PLUGIN UR_ROBOT_DRIVER PER MOVEIT
# ================================================
MOVEIT_CONFIG_DIR="${WS_DIR}/src/moveit_config/config"

echo "Aggiornamento del plugin ur_robot_driver nel file ur5e.ros2_control.xacro..."
python3 ${MOVEIT_CONFIG_DIR}/ros2_control_hardware_auto_switch.py driver --robot_ip="${robot_ip}" --reverse_ip="${reverse_ip}"
#------------------------------------------------


# ================================================
# lANCIO DEI CONTAINER
# ================================================
if [[ "$launch_container" == "true" ]]; then
    echo -e "\nAvvio URSim in un container Docker...\n"

    # Forza la chiusura e la rimozione di un eventuale container precedente
    echo "Pulizia di vecchi container URSim in corso..."
    docker rm -f ursim >/dev/null 2>&1 || true

    # Avvia il container URSim
    gnome-terminal --tab --title="URSim Launcher" -- bash -c "ros2 run ur_client_library start_ursim.sh; exec bash"
    sleep 5
else
    echo -e "\nIl container non è stato lanciato. Assicurati che sia già in esecuzione all'IP 192.168.56.101."
fi




# ================================================
# AVVIO DRIVER UR5e 
# ================================================
if [[ "$launch_robot_driver" == "true" ]]; then
    
    #sincronizzazione con l'utente
    echo -e "\n================================================================="
    echo -e "ATTENZIONE: ATTESA DI SINCRONIZZAZIONE CON L'UTENTE"
    echo -e "================================================================="
    echo -e "Verifica che il robot simulato sia pronto per l'operatività.\nDevi averlo acceso cliccando con il mouse sul pulsante \"Power Off\" e diventato verde \"Normal\"\n"
    echo -e "Premi un tasto per avviare i driver..."

    # Mettiamo in pausa in attesa del segnale
    read -n 1 -s -r

    echo -e "\nProcedo con l'avvio del driver UR5e..."


    # Calcolo il path base del pacchetto arm_description una volta sola
    # (Funziona perché hai già fatto 'source "${INSTALL_SETUP_BASH}"' in cima allo script)
    ARM_DESC_DIR="$(ros2 pkg prefix arm_description)/share/arm_description"

    gnome-terminal --tab --title="Driver UR5e" -- bash -c "source \"${INSTALL_SETUP_BASH}\" && \
        ros2 launch ur_robot_driver ur_control.launch.py \
            ur_type:=ur5e \
            robot_ip:=${URSIM_ROBOT_IP} \
            reverse_ip:=${UR_SIM_REVERSE_IP} \
            launch_rviz:=false \
            description_file:=${ARM_DESC_DIR}/urdf/arm_driver_wrapper.urdf.xacro; \
        exec bash"

else
    echo -e "\nDriver UR5e non avviati da questo script. Assicurati che siano già attivi..."
fi



# ================================================
# AVVIO MOVEIT
# ================================================
echo -e "\n================================================================="
echo -e "ATTENZIONE: ATTESA DI SINCRONIZZAZIONE CON L'UTENTE"
echo -e "================================================================="
echo -e "Verifica che il driver sia partito correttamente.\nDevi aver selezionato \"External control\" dal teach pendant e aver premuto Play.\nDal terminale del driver dovresti vedere la scritta 'Robot ready to receive commands'.\n"
echo -e "Premi un tasto per avviare Moveit..."

# Mettiamo in pausa in attesa del segnale
read -n 1 -s -r


echo "Avvio MoveIt..."
gnome-terminal --tab --title="MoveGroup" -- bash -c \
                "source ${INSTALL_SETUP_BASH} && \
                ros2 launch moveit_config move_group.launch.py \
                    use_sim_time:=false; \
                exec bash"
sleep 5


echo "Avvio RViz..."
gnome-terminal --tab --title="Rviz" -- bash -c \
               "source ${INSTALL_SETUP_BASH} && \
                ros2 launch moveit_config moveit_rviz.launch.py \
                    use_sim_time:=false; \
                exec bash"
sleep 5


# ================================================
# AVVIO DETECTION CON CAMERA SIMULATA (FAKE CAMERA)
# ================================================
echo "Avvio Fake Camera..."
gnome-terminal --tab --title="Fake Camera" -- bash -c \
                "source ${INSTALL_SETUP_BASH} && \
                ros2 launch fake_camera fake_camera.launch.py; \
                exec bash"



# ================================================
# BUILDER DELLA SCENA
# ================================================
echo "Avvio Scene Builder..."
gnome-terminal --tab --title="Scene Builder" -- bash -c \
                "source ${INSTALL_SETUP_BASH} && \
                ros2 run scene_description scene_builder; \
                exec bash"




# ================================================
# ESECUZIONE
# ================================================
if [[ "$execute_shot" == "true" ]]; then

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

    
    #eseguo il nodo di shot planning con i parametri definiti
    echo "Avvio Shot Planning..."
    gnome-terminal --tab --title="Shot Planning" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 run shot_planning task_node ${NODE_ARGS}; \
                    exec bash"
    

    # ================================================
    # GAME ENGINE (REALE)
    # ================================================
    GAME_ENGINE_CONFIG_DIR="${WS_DIR}/src/shot_execution/game_engine/config"

    echo "Avvio Game Engine Reale..."
    gnome-terminal --tab --title="Game Engine Reale" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 run game_engine game_engine --ros-args \
                        --params-file ${GAME_ENGINE_CONFIG_DIR}/game_engine_params.yaml; \
                    exec bash"
    
    
fi


echo "Tutti i nodi sono stati avviati!"
