#!/bin/bash


#------------------------------------------------
# PARAMETRI DI SCRIPT PER LA LEGGIBILITA' E LA MANUTENIBILITA'
WS_DIR="ws_ur5e_ballpool"
INSTALL_DIR="${WS_DIR}/install"
INSTALL_SETUP_BASH="${WS_DIR}/install/setup.bash"
#------------------------------------------------



# ------------------------------------------------
# PARAMETRI DI PERSONALIZZAZIONE ESECUZIONE OFF-LINE

#simulazione
open_rviz_when_using_mujoco="true"            #true se vuoi aprire anche RViz quando usi MuJoCo, false se vuoi aprire solo MuJoCo

#scena e detection
use_vision_node_for_mujoco="true"              #true se vuoi usare il nodo di vision, false se vuoi usare la scena fake con le palline già posizionate (fake camera) (solo con MuJoCo, altrimenti non esiste la telecamera simulata)
start_image_view="false"                       #true se vuoi avviare image_view per visualizzare il feed della camera, false se non vuoi avviarlo (solo con MuJoCo, altrimenti non esiste la telecamera simulata)
build_scene_rviz="true"                        #true se vuoi costruire la scena in RViz, indicando gli ostacoli in moveit

#esecuzione tiro
execute_shot="true"                            #false se vuoi solo fare visualizzazione della scena e non eseguire il tiro (utile in fase di debug e setup)
use_real_game_engine="true"                    #true se vuoi usare il game engine reale, false se vuoi usare quello fake

#logging
logging_enable="false"                                        #true se vuoi fare logging

only_essential_logging="false"                                #true se vuoi fare logging solo dei dati essenziali, false se vuoi fare logging di tutti i dati
only_essential_logging_folder="only_essential_logging"        #nome della cartella di logging, che verrà creata in data/bagdata/<logging_folder_title>

only_camera_logging="true"                                   #true se vuoi fare logging solo dei dati della camera, false se vuoi fare logging di tutti i dati                                        
only_camera_logging_folder="only_camera_logging"              #nome della cartella di logging, che verrà creata in data/bagdata/<logging_folder_title>

brutal_logging="false"                                        #true se vuoi fare logging di tutti i dati, false se vuoi fare logging solo dei dati essenziali
brutal_logging_folder="brutal_logging"                        #nome della cartella di logging, che verrà creata in data/bagdata/<logging_folder_title>

# ------------------------------------------------


# ------------------------------------------------
# Verifica preliminare della cartella di lavoro
if [ ! -d "${INSTALL_DIR}" ]; then
    echo "Errore: Cartella 'install' non trovata!"
    echo "Assicurati di aver buildato il workspace ROS 2 e di lanciare questo script dalla root del progetto."
    exit 1
fi
# ------------------------------------------------


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

    # aggiornamento del plugin di MuJoCo nel file ur5e.ros2_control.xacro
    MOVEIT_CONFIG_DIR="${WS_DIR}/src/moveit_config/config"

    echo "Aggiornamento del plugin MuJoCo nel file ur5e.ros2_control.xacro..."
    python3 ${MOVEIT_CONFIG_DIR}/ros2_control_hardware_auto_switch.py mujoco


    # devo generare la scena completa con le palline per MuJoCo con lo script autocreate_complete_scene.py
    MUJOCO_AUTOMATION_DIR="${WS_DIR}/src/camera_perception/fake_camera/mujoco_automation"
    FAKE_CAMERA_CONFIG_DIR="${WS_DIR}/src/camera_perception/fake_camera/config"

    echo "Aggiornamento del file della scena MuJoCo con posizione delle palline..."
    python3 ${MUJOCO_AUTOMATION_DIR}/autocreate_complete_scene.py --yaml_path ${FAKE_CAMERA_CONFIG_DIR}/fake_camera_config.yaml
            

    #avvio del simulatore vero e proprio con MuJoCo e MoveIt
    echo "Avvio MuJoCo con MoveIt..."
    sleep 2

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
fi


# ================================================
# AVVIO PERCEZIONE SIMULATA
# ================================================
# modalità:
#   - uso la camera simulata in MuJoCo (use_vision_node=true e scelta_mujoco=s)
#   - uso la scena ideale con le terne messe da fake_camera (use_vision_node=false)

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
        sleep 1
    fi
   
    #avvio nodo di visione che effettua la detection e la perception
    echo "Avvio nodo di visione..."
    gnome-terminal --tab --title="VisionNode" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 launch real_camera vision.launch.py \
                    use_sim_time:=true; \
                    exec bash"
    sleep 1
else
    # NOTA: se uso la scena ideale con le terne messe da fake_camera, lancio il nodo che pubblica idealmente già la posizione da file yaml

    FAKE_CAMERA_CONFIG_DIR="${WS_DIR}/src/camera_perception/fake_camera/config"

    echo "Avvio Fake Camera..."
    gnome-terminal --tab --title="Fake Camera" -- bash -c \
                        "source ${INSTALL_SETUP_BASH} && \
                        ros2 launch fake_camera fake_camera.launch.py \
                            yaml_path:=${FAKE_CAMERA_CONFIG_DIR}/fake_camera_config.yaml; \
                        exec bash"
    sleep 2
    
fi


# ================================================
# BUILDER DELLA SCENA
# ================================================
if [[ "$build_scene_rviz" == "true" ]]; then
    echo "Avvio Scene Builder..."
    gnome-terminal --tab --title="Scene Builder" -- bash -c \
                        "source ${INSTALL_SETUP_BASH} && \
                        ros2 run scene_description scene_builder; \
                        exec bash"

    sleep 2
fi


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
        NODE_ARGS="${NODE_ARGS} --params-file ${SHOT_CONFIG_DIR}/essential_logging_params.yaml"
    fi

    # Aggiungo il file di MuJoCo se richiesto
    if [[ "$scelta_mujoco" =~ ^[sS][iI]?$ ]]; then
        NODE_ARGS="${NODE_ARGS} --params-file ${SHOT_CONFIG_DIR}/using_mujoco.yaml"
    fi

 
    #eseguo il nodo di shot planning con i parametri definiti
    echo "Avvio Shot Planning..."
    gnome-terminal --tab --title="Shot Planning" -- bash -c " \
        source ${INSTALL_SETUP_BASH} && \
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