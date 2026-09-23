#!/bin/bash

#------------------------------------------------
# PARAMETRI DI SCRIPT PER LA LEGGIBILITA' E LA MANUTENIBILITA'
WS_DIR="ws_ur5e_ballpool"
INSTALL_DIR="${WS_DIR}/install"
INSTALL_SETUP_BASH="${WS_DIR}/install/setup.bash"
#------------------------------------------------


# ------------------------------------------------
# PARAMETRI DI RETE
LAB_ROBOT_IP="192.168.1.110"

URSIM_ROBOT_IP="192.168.56.101"
UR_SIM_REVERSE_IP="192.168.56.1"  #il tuo ip sulla rete del robot simulato
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
execute_shot="false"                           
use_real_game_engine="true"                   

#logging
logging_enable="false"                                        

only_essential_logging="false"                                
only_essential_logging_folder="only_essential_logging"        

only_camera_logging="false"                                   
only_camera_logging_folder="only_camera_logging"              

brutal_logging="false"                                        
brutal_logging_folder="brutal_logging"                        
# ------------------------------------------------


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
read -p "Avviare in simulazione con URSim? (s/n): " scelta_URSim


if [[ "$scelta_URSim" =~ ^[sS][iI]?$ ]]; then
    echo "Hai scelto di avviare in simulazione con URSim."

    cat << 'EOF'

Sequenza corretta (Simulazione):

1. Avvio Container Docker (URSim) ✅
    - Lo script lancia il container
    - Attendi che si apra l'interfaccia URSim (all'ip 192.168.56.101)
2. Accensione Robot Virtuale ✅
    - Nel teach pendant di URSim: clicca "Power Off" → "Normal"
    - Attendi che il robot si inizializzi (led diventa verde)
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

else
    echo "Hai scelto di avviare senza URSim (robot reale)."

    cat << 'EOF'

Sequenza corretta (Robot Reale):

1. Preparazione Robot Fisico ✅
    - Accendi il controller e sblocca i giunti (rilascia i freni, led verde)
    - Assicurati che il cavo di rete sia collegato tra PC e controller
2. Avvio Driver ✅
    - Il driver parte e si connette all'IP del robot reale
    - MA il robot NON è ancora pronto a ricevere comandi!
3. Avvio External Control Program ⚠️ PASSAGGIO CRUCIALE
    - Sul Teach Pendant fisico del robot:
        - Apri/Crea un programma contenente il nodo "External Control"
    - Verifica in "Installation" che l'IP dell'Host (il tuo PC) sia corretto
    - Premi "Play" ▶️
    - Solo ora il robot accetta comandi dal driver
    - Lo script aspetta che tu prema un tasto dopo aver visto "Robot ready to receive commands"
4. Avvio MoveIt ✅
    - Ora MoveIt può comandare il robot fisico

Quindi: Accendi Robot → Driver → External Control → MoveIt 🎯
EOF

fi

sleep 3


# ================================================
# AGGIORNAMENTO PLUGIN UR_ROBOT_DRIVER PER MOVEIT
# ================================================
if [[ "$scelta_URSim" =~ ^[sS][iI]?$ ]]; then
    #robot simulato
    robot_ip="${URSIM_ROBOT_IP}"
    reverse_ip="${UR_SIM_REVERSE_IP}"
else
    #robot reale
    robot_ip="${LAB_ROBOT_IP}"
    reverse_ip=""  #non serve per il robot reale
fi


MOVEIT_CONFIG_DIR="${WS_DIR}/src/moveit_config/config"

echo "Aggiornamento del plugin ur_robot_driver nel file ur5e.ros2_control.xacro..."
python3 ${MOVEIT_CONFIG_DIR}/ros2_control_hardware_auto_switch.py driver --robot_ip="${robot_ip}" --reverse_ip="${reverse_ip}"
#------------------------------------------------


# ================================================
# lANCIO DEI CONTAINER NEL CASO DELLA SCELTA IN SIMULAZIONE CON URSIM
# ================================================
if [[ "$scelta_URSim" =~ ^[sS][iI]?$ ]]; then

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

fi



# ================================================
# AVVIO DRIVER UR5e 
# ================================================
if [[ "$launch_robot_driver" == "true" ]]; then
    
    #sincronizzazione con l'utente
    echo -e "\n================================================================="
    echo -e "ATTENZIONE: ATTESA DI SINCRONIZZAZIONE CON L'UTENTE"
    echo -e "================================================================="
    echo -e "Verifica che il robot (simulato o reale) sia pronto per l'operatività.\nDevi averlo acceso cliccando con il mouse sul pulsante \"Power Off\" e diventare verde \"Normal\"\n"
    echo -e "Premi un tasto per avviare i driver..."

    # Mettiamo in pausa in attesa del segnale
    read -n 1 -s -r

    echo -e "\nProcedo con l'avvio del driver UR5e..."


    # Calcolo il path base del pacchetto arm_description una volta sola
    # (Funziona perché hai già fatto 'source "${INSTALL_SETUP_BASH}"' in cima allo script)
    ARM_DESC_DIR="$(ros2 pkg prefix arm_description)/share/arm_description"

    # DIFFERENZE ROBOT SIMULATO E REALE
    #   -   cambia l'ip del robot
    #   -   cambia il reverse_ip (solo per simulazione)
    #   -   cambia il file di kinematics_params_file (solo per robot reale)

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
    
    sleep 3
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
# AVVIO DETECTION CON CAMERA (O SIMULATO TUTTO CON FAKE CAMERA)
# ================================================
if [[ "$use_real_camera" == "true" ]]; then

    #avvio driver della camera
    echo "Avvio driver IntelRealSense..."
    gnome-terminal --tab --title="Intel Realsense Camera" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 launch realsense2_camera rs_launch.py \
                            enable_rgbd:=true \
                            enable_sync:=true \
                            rgb_camera.color_profile:=1280x720x15 \
                            depth_module.depth_profile:=1280x720x15 \
                            align_depth.enable:=true \
                            pointcloud.enable:=true \
                            spatial_filter.enable:=true \
                            spatial_filter.filter_magnitude:=3 \
                            spatial_filter.filter_smooth_alpha:=0.3 \
                            spatial_filter.filter_smooth_delta:=20 \
                            temporal_filter.enable:=true \
                            temporal_filter.filter_smooth_alpha:=0.1 \
                            temporal_filter.filter_smooth_delta:=20 \
                            temporal_filter.filter_persistency:=8 \
                            hole_filling_filter.enable:=true; \
                        exec bash"
    sleep 3
    
    #image_view per visualizzare il feed della camera
    if [ "$start_image_view" == "true" ]; then
        echo "Avvio image_view..."
        gnome-terminal --tab --title="image_view" -- bash -c \
                            "source ${INSTALL_SETUP_BASH} && \
                            ros2 run image_view image_view --ros-args -r \
                                image:=/camera/camera/color/image_raw; \
                            exec bash"
        sleep 1
    fi

    #avvio nodo di visione che effettua la detection e la perception
    echo "Avvio nodo di visione..."
    gnome-terminal --tab --title="VisionNode" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 launch real_camera vision.launch.py; \
                    exec bash"
    sleep 1

else
    
    #faccio pubblicare le terne 'finte' alla fake camera
    echo "Avvio Fake Camera..."
    gnome-terminal --tab --title="Fake Camera" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 launch fake_camera fake_camera.launch.py; \
                    exec bash"
    sleep 2
fi


# ================================================
# BUILDER DELLA SCENA
# ================================================
echo "Avvio Scene Builder..."
gnome-terminal --tab --title="Scene Builder" -- bash -c \
                "source ${INSTALL_SETUP_BASH} && \
                ros2 run scene_description scene_builder; \
                exec bash"

sleep 2

ros2 service call /build_scene std_srvs/srv/Trigger "{}"


# ================================================
# ESECUZIONE
# ================================================
if [[ "$execute_shot" == "true" ]]; then

    # ================================================
    # GESTIONE DEL LOGGING
    # ================================================
    if [[ "$logging_enable" == "true" ]]; then

        BAGDATA_DIR="data/bagdata"

        if [[ "$only_essential_logging" == "true" ]]; then

            # ------------------------------------------------
            # solo logging essenziale, topic principali durante il tiro


            #avvio il nodo di debug cartesiano che pubblica la posa del TCP del robot
            echo "Avvio Nodo di debug cartesiano..."
            gnome-terminal --tab --title="CartesianPublisher" -- bash -c \
                           "source ${INSTALL_SETUP_BASH} && \
                           ros2 launch logging_nodes cartesian_pub_launcher.launch.py; \
                           exec bash"
            sleep 1


            # solo logging essenziale, topic principali durante il tiro
            echo "Avvio Bag Writer essenziale su richiesta..."
            gnome-terminal --tab --title="Logging nodes" -- bash -c \
                           "source ${INSTALL_SETUP_BASH} && \
                           ros2 launch logging_nodes bag_writer.launch.py \
                                test_title:='${only_essential_logging_folder}'; \
                            exec bash"
            sleep 1
            # ------------------------------------------------
        fi


        if [[ "$only_camera_logging" == "true" ]]; then

            # ------------------------------------------------
            # solo logging dei topic della camera, durante l'esecuzione di tutto il programma


            BAGDATA_DIR_CAMERA="${BAGDATA_DIR}/${only_camera_logging_folder}"

            #cancello la cartella di logging precedente se esiste, così da non avere conflitti
            rm -rf "${BAGDATA_DIR_CAMERA}"  

            # solo logging della camera
            echo "ros2 bag record dei topic della camera..."
            gnome-terminal --tab --title="ros2bag camera record" -- \
                bash -c "source \"${INSTALL_SETUP_BASH}\" && \
                ros2 bag record -o ${BAGDATA_DIR_CAMERA} \
                    /camera/camera/color/camera_info \
                    /camera/camera/color/image_raw \
                    /camera/camera/aligned_depth_to_color/image_raw; \
                exec bash"

            sleep 2
            # ------------------------------------------------
        fi

        
        if [[ "$brutal_logging" == "true" ]]; then
    
            # ------------------------------------------------
            # logging di tutti i topic, durante l'esecuzione di tutto il programma


            #avvio il nodo di debug cartesiano che pubblica la posa del TCP del robot
            echo "Avvio Nodo di debug cartesiano..."
            gnome-terminal --tab --title="CartesianPublisher" -- bash -c \
                           "source ${INSTALL_SETUP_BASH} && \
                           ros2 launch logging_nodes cartesian_pub_launcher.launch.py; \
                           exec bash"
            sleep 1


            BAGDATA_DIR_BRUTAL="${BAGDATA_DIR}/${brutal_logging_folder}"

            #cancello la cartella di logging precedente se esiste, così da non avere conflitti
            rm -rf "${BAGDATA_DIR_BRUTAL}"  

            echo "Avvio ros2 bag record manuale..."
            gnome-terminal --tab --title="ros2bag brutal record" -- bash -c \
                            "source ${INSTALL_SETUP_BASH} && \
                            ros2 bag record -o ${BAGDATA_DIR_BRUTAL} -a; \
                            exec bash"

            sleep 2

        fi
    fi
    

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

    # Aggiungo il file di essential_logging se richiesto
    if [[ "$logging_enable" == "true" && "$only_essential_logging" == "true" ]]; then
        NODE_ARGS="${NODE_ARGS} --params-file ${SHOT_CONFIG_DIR}/essential_logging_params.yaml"     #parametro aggiunto
    fi

    #eseguo il nodo di shot planning con i parametri definiti
    echo "Avvio Shot Planning..."
    gnome-terminal --tab --title="Shot Planning" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 run shot_planning task_node ${NODE_ARGS}; \
                    exec bash"
    sleep 5
    

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
    
fi


echo "Tutti i nodi sono stati avviati!"