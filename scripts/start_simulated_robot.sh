#!/bin/bash

#------------------------------------------------
# DESCRIZIONE SCRIPT

# Questo script serve per avviare tutti i nodi necessari per la simulazione del robot
# e per l'esecuzione del tiro, sia in modalità MuJoCo che in modalità Fake Hardware.
# Lo script permette di scegliere se usare MuJoCo o Fake Hardware, se usare la telecamera simulata 
# o la scena ideale, se aprire RViz o meno, se fare logging e quale tipo di logging fare.
#------------------------------------------------


# ------------------------------------------------
# PARAMETRI DI PERSONALIZZAZIONE ESECUZIONE OFF-LINE

#simulazione
open_rviz_when_using_mujoco="true"            #true se vuoi aprire anche RViz quando usi MuJoCo, false se vuoi aprire solo MuJoCo

#scena e detection
graphical_input_scene="true"                   #true se vuoi usare la scena grafica per posizionare le palline, false se vuoi usare il file di config scritto a mano (fake camera)
use_vision_node_for_mujoco="true"              #(*) true se vuoi usare il nodo di vision che fa detection della telecamera di mujoco, false se vuoi usare la scena fake con le palline già posizionate (fake camera)
numero_campioni_detection_biliardo=10         #(*) numero di campioni da utilizzare per la detection media del biliardo, dalla telecamera di mujoco (utile solo se true camera di mujoco)
start_image_view="false"                       #(*) true se vuoi avviare image_view per visualizzare il feed della camera, false se non vuoi avviarlo 
start_image_view="false"                       #(*) true se vuoi avviare image_view per visualizzare il feed della camera, false se non vuoi avviarlo (solo con MuJoCo, altrimenti non esiste la telecamera simulata)
# (*) = solo in modalità MuJoCo

#esecuzione tiro
execute_shot="true"                            #false se vuoi solo fare visualizzazione della scena e non eseguire il tiro (utile in fase di debug e setup)
use_real_game_engine="true"                    #true se vuoi usare il game engine reale, false se vuoi usare quello fake

#logging
logging_enable="true"                                        #true se vuoi fare logging
only_essential_logging="true"                                #true se vuoi fare logging solo dei dati essenziali, false se vuoi fare logging di tutti i dati
only_essential_logging_folder="only_essential_logging"       #nome della cartella di logging, che verrà creata in data/bagdata/<logging_folder_title>

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
#se false, legge il file di config, puoi scriverlo a mano (fake_camera_config.py)
FAKE_CAMERA_CONFIG_DIR="${WS_DIR}/src/camera_perception/fake_camera/config"

if [[ "$graphical_input_scene" == "true" ]]; then
    # Avvio dell'interfaccia grafica per la configurazione della scena
    echo "Avvio dell'interfaccia grafica per la configurazione della scena..."
    python3 ${FAKE_CAMERA_CONFIG_DIR}/graphical_config_generator.py
    echo -e "Configurazione della scena completata.\n"
fi


# ================================================
# AVVIO SIMULATORI + MOVEIT
# ================================================
echo "========================================"
echo "      CONFIGURAZIONE AVVIO ROS 2        "
echo "========================================"

read -p "Vuoi usare MuJoCo? (s/n): " scelta_mujoco

if [[ "$scelta_mujoco" =~ ^[sS][iI]?$ ]]; then

    #------------------------------------------------
    # gestione del setup di MuJoCo

    #variabile che userò nello script per gestire sim_time
    scelta_mujoco_bool="true"
    echo -e "\nUtilizzo di MuJoCo."

    # aggiornamento del plugin di MuJoCo nel file ur5e.ros2_control.xacro
    MOVEIT_CONFIG_DIR="${WS_DIR}/src/moveit_config/config"

    echo -e "\nAggiornamento del plugin MuJoCo nel file ur5e.ros2_control.xacro..."
    python3 ${MOVEIT_CONFIG_DIR}/ros2_control_hardware_auto_switch.py mujoco


    # devo generare la scena completa con le palline per MuJoCo con lo script autocreate_complete_scene.py
    MUJOCO_AUTOMATION_DIR="${WS_DIR}/src/camera_perception/fake_camera/mujoco_automation"
    FAKE_CAMERA_CONFIG_DIR="${WS_DIR}/src/camera_perception/fake_camera/config"

    echo "Aggiornamento del file della scena MuJoCo con posizione delle palline..."
    python3 ${MUJOCO_AUTOMATION_DIR}/autocreate_complete_scene.py --yaml_path ${FAKE_CAMERA_CONFIG_DIR}/fake_camera_config.yaml
            

    #avvio del simulatore vero e proprio con MuJoCo e MoveIt
    echo "Avvio MuJoCo con MoveIt..."
    sleep 1

    if [[ "$open_rviz_when_using_mujoco" == "true" ]]; then
        gnome-terminal --tab --title="Moveit+MuJoCo+Rviz" -- bash -c \
                        "source ${INSTALL_SETUP_BASH} && \
                        ros2 launch moveit_config mujoco_and_rviz_demo.launch.py; \
                        exec bash"
        sleep 10
    else
        gnome-terminal --tab --title="Moveit+MuJoCo" -- bash -c \
                        "source ${INSTALL_SETUP_BASH} && \
                        ros2 launch moveit_config mujoco_demo.launch.py; \
                        exec bash"
        sleep 10
    fi

    #------------------------------------------------
else

    #------------------------------------------------
    # gestione del setup di Fake Hardware

    #variabile che userò nello script per gestire sim_time
    scelta_mujoco_bool="false"

    # aggiornamento del plugin di MockHardware nel file ur5e.ros2_control.xacro
    MOVEIT_CONFIG_DIR="${WS_DIR}/src/moveit_config/config"

    echo "Aggiornamento del plugin MockHardware nel file ur5e.ros2_control.xacro..."
    python3 ${MOVEIT_CONFIG_DIR}/ros2_control_hardware_auto_switch.py mock

    echo "Avvio MoveIt con RViz..."

    gnome-terminal --tab --title="MoveIt+Rviz" -- bash -c \
                        "source ${INSTALL_SETUP_BASH} && \
                        ros2 launch moveit_config demo.launch.py; \
                        exec bash"
    sleep 10
    #------------------------------------------------
fi


# ================================================
# AVVIO PERCEZIONE SIMULATA
# ================================================
# modalità:
#   - uso la camera simulata in MuJoCo (use_vision_node=true e scelta_mujoco=s)
#   - uso la scena ideale con le terne messe da fake_camera (else)

if [[ "$use_vision_node_for_mujoco" == "true" && "$scelta_mujoco" =~ ^[sS][iI]?$ ]]; then

    # avvio detection da telecamera simulata in MuJoCo
    # NOTA: se uso MuJoCo, la telecamera simulata è già presente, quindi lancio il nodo di vision che legge i topic della camera e pubblica la posizione delle palline

    #image_view per visualizzare il feed della camera
    if [ "$start_image_view" == "true" ]; then
        echo "Avvio image_view..."
        gnome-terminal --tab --title="image_view" -- bash -c \
                        "source ${INSTALL_SETUP_BASH} && \
                        ros2 run image_view image_view --ros-args -r \
                            image:=/camera/camera/color/image_raw; \
                        exec bash"
    fi
   
    #avvio nodo di visione che effettua la detection e la perception
    echo "Avvio nodo di visione + freezer node..."
    gnome-terminal --tab --title="VisionNode" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 launch real_camera vision.launch.py \
                        required_samples:=${numero_campioni_detection_biliardo}; \
                        use_sim_time:=true; \
                    exec bash"
else
    # NOTA: se uso la scena ideale con le terne messe da fake_camera, lancio il nodo che pubblica idealmente già la posizione da file yaml

    FAKE_CAMERA_CONFIG_DIR="${WS_DIR}/src/camera_perception/fake_camera/config"

    echo "Avvio Fake Camera..."
    gnome-terminal --tab --title="Fake Camera" -- bash -c \
                        "source ${INSTALL_SETUP_BASH} && \
                        ros2 launch fake_camera fake_camera.launch.py \
                            yaml_path:=${FAKE_CAMERA_CONFIG_DIR}/fake_camera_config.yaml; \
                        exec bash"


    fake_camera_usage="true"
    only_camera_logging="false"  #se uso la fake camera, non ha senso fare logging solo della camera, perché non c'è una camera reale
fi


# ================================================
# BUILDER DELLA SCENA
# ================================================
echo "Avvio Scene Builder..."
gnome-terminal --tab --title="Scene Builder" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 run scene_description scene_builder --ros-args \
                        -p use_sim_time:=${scelta_mujoco_bool}; \
                    exec bash"


# ================================================
# ESECUZIONE
# ================================================
if [[ "$execute_shot" == "true" ]]; then

   
    # ================================================
    # GESTIONE DEL LOGGING
    # ================================================
    if [[ "$only_essential_logging" == "true" ]]; then

        BAGDATA_DIR="data/bagdata"

        
        echo "Avvio Nodo di pubblicazione cartesiana..."
        gnome-terminal --tab --title="Cartesian Pose Publisher TCP" -- bash -c \
                        "source ${INSTALL_SETUP_BASH} && \
                        ros2 launch logging_nodes cartesian_pose_pub.launch.py \
                            use_sim_time:=${scelta_mujoco_bool}; \
                        exec bash"


        echo "Avvio Nodo di pubblicazione twist..."
        gnome-terminal --tab --title="Cartesian Twist Publisher TCP" -- bash -c \
                        "source ${INSTALL_SETUP_BASH} && \
                        ros2 launch logging_nodes cartesian_twist_pub.launch.py \
                            use_sim_time:=${scelta_mujoco_bool}; \
                        exec bash"

        # ------------------------------------------------
        # solo logging essenziale, topic principali durante il tiro

        echo "Avvio Bag Writer essenziale su richiesta..."
        gnome-terminal --tab --title="Logging nodes" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 launch logging_nodes bag_writer.launch.py \
                            test_title:='${only_essential_logging_folder}' \
                            use_sim_time:=${scelta_mujoco_bool}; \
                        exec bash"
        # ------------------------------------------------
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
        NODE_ARGS="${NODE_ARGS} --params-file ${SHOT_CONFIG_DIR}/essential_logging_params.yaml"
    fi

    # Aggiungo il file di Fake Camera se richiesto
    if [[ "$fake_camera_usage" == "true" ]]; then
        NODE_ARGS="${NODE_ARGS} --params-file ${SHOT_CONFIG_DIR}/using_fake_camera.yaml"
    fi

    # Aggiungo il parametro use_sim_time se sto usando MuJoCo
    if [[ "$scelta_mujoco_bool" == "true"  ]]; then
        NODE_ARGS="${NODE_ARGS} -p use_sim_time:=true"
    fi

 
    #eseguo il nodo di shot planning con i parametri definiti
    echo "Avvio Shot Planning..."
    gnome-terminal --tab --title="Shot Planning" -- bash -c " \
        source ${INSTALL_SETUP_BASH} && \
        ros2 run shot_planning task_node ${NODE_ARGS}; \
        exec bash"
    

    # ================================================
    # GAME ENGINE (REALE O SIMULATO)
    # ================================================
    GAME_ENGINE_CONFIG_DIR="${WS_DIR}/src/shot_execution/game_engine/config"

    if [[ "$use_real_game_engine" == "true" ]]; then
        
        echo "Avvio Game Engine Reale..."
        gnome-terminal --tab --title="Game Engine Reale" -- bash -c \
                       "source ${INSTALL_SETUP_BASH} && \
                       ros2 run game_engine game_engine --ros-args \
                           --params-file ${GAME_ENGINE_CONFIG_DIR}/game_engine_params.yaml \
                           -p use_sim_time:=${scelta_mujoco_bool}; \
                       exec bash"
    else
        echo "Avvio Fake Game Engine..."
        gnome-terminal --tab --title="Fake Game Engine" -- bash -c \
                        "source ${INSTALL_SETUP_BASH} && \
                        ros2 run game_engine fake_game_engine --ros-args \
                            --params-file ${GAME_ENGINE_CONFIG_DIR}/fake_game_engine_params.yaml \
                            -p use_sim_time:=${scelta_mujoco_bool}; \
                        exec bash"
        
    fi
fi


echo "Tutti i nodi sono stati avviati!"