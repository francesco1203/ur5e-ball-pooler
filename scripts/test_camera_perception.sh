#!/bin/bash

#------------------------------------------------
# PARAMETRI DI SCRIPT PER LA LEGGIBILITA' E LA MANUTENIBILITA'
WS_DIR="ws_ur5e_ballpool"
INSTALL_DIR="${WS_DIR}/install"
INSTALL_SETUP_BASH="${WS_DIR}/install/setup.bash"
#------------------------------------------------

# ------------------------------------------------
# PARAMETRI DI PERSONALIZZAZIONE ESECUZIONE OFF-LINE

use_real_camera="true"                  # se true, si usa la camera reale, altrimenti si usano strumenti simulati
launch_driver="true"                    # solo se use_real_camera è true, altrimenti non serve             

#strumenti simulati (se non si usa la camera reale)
use_camera_bagfiles_stream="false"                          
CAMERA_BAGFILES_FOLDER="data/bagdata/camera_stream"   
CAMERA_BAGFILE_PATH="${CAMERA_BAGFILES_FOLDER}/..."         

use_mujoco_camera="false"                        
use_fake_camera_node="false"                     
use_prefix_for_fake_camera="fake"                

#simulatori per la percezione
start_image_view="false"                        
start_Rviz="true"                                 

#parte vision
launch_vision_node="false"                   

#costruzione scena
launch_scene_builder="false"                  
auto_loop_build_scene="true"                   
time_between_scene_builds=1                    
user_input_to_build_scene="false"              # Se auto_loop è true, meglio tenere questo a false (sono esclusivi nella logica sotto)
# ------------------------------------------------

# ------------------------------------------------
# Verifica preliminare della cartella di lavoro
if [ ! -d "${INSTALL_DIR}" ]; then
    echo "Errore: Cartella 'install' non trovata!"
    echo "Assicurati di lanciare questo script dalla root del workspace ROS 2."
    exit 1
fi
# ------------------------------------------------


# ================================================
# LOGICA DI LANCIO CAMERA (REALE VS SIMULATA)
# ================================================
if [ "$use_real_camera" == "true" ]; then

    #------------------------------------------------
    # chiusura eventuali nodi realsense2_camera_node già in esecuzione
    echo "Chiudo eventuali nodi realsense2_camera_node già in esecuzione..."
    killall -9 realsense2_camera_node


    # ------------------------------------------------
    # avvio driver per la camera reale per stream live
    if [ "$launch_driver" == "true" ]; then
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
                              temporal_filter.enable:=true; \
                         exec bash"
        sleep 3
    else
        echo -e "Driver IntelRealSense non avviati da questo script. Assicurati che siano già attivi..."
    fi
    # ------------------------------------------------

else
    #strumenti simulati

    if [ "$use_camera_bagfiles_stream" == "true" ]; then
        echo "Avvio stream da bag files..."
        gnome-terminal --tab --title="BagFile Player" -- bash -c \
                       "source ${INSTALL_SETUP_BASH} && \
                       ros2 bag play ${CAMERA_BAGFILE_PATH} ; \
                       exec bash"
        sleep 1

    fi

    if [ "$use_mujoco_camera" == "true" ]; then
        echo "Aggiornamento del plugin MuJoCo nel file ur5e.ros2_control.xacro..."
        python3 ${WS_DIR}/src/moveit_config/config/ros2_control_hardware_auto_switch.py mujoco

        echo "Aggiornamento del file della scena MuJoCo con posizione delle palline..."
        python3 ${WS_DIR}/src/camera_perception/fake_camera/mujoco_automation/autocreate_complete_scene.py --yaml_path ${WS_DIR}/src/camera_perception/fake_camera/config/fake_camera_config.yaml
                
        echo "Avvio scena con camera simulata in MuJoCo..."
        gnome-terminal --tab --title="MuJoCo Simulation" -- bash -c \
                       "source ${INSTALL_SETUP_BASH} && \
                       ros2 launch fake_camera mujoco_with_stream.launch.py; \
                       exec bash"
        sleep 2

    fi

    if [ "$use_fake_camera_node" == "true" ]; then
        if [ "$use_prefix_for_fake_camera" == "true" ]; then
            echo "Avvio fake camera con prefisso per i frame..."
            gnome-terminal --tab --title="Fake Camera" -- bash -c \
                           "source ${INSTALL_SETUP_BASH} && \
                           ros2 launch fake_camera fake_camera.launch.py \
                                prefix:=fake_; \
                           exec bash"
        else
            echo "Avvio fake camera senza prefisso per i frame..."
            gnome-terminal --tab --title="Fake Camera" -- bash -c \
                           "source ${INSTALL_SETUP_BASH} && \
                           ros2 launch fake_camera fake_camera.launch.py; \
                           exec bash"
        fi
        sleep 1
    fi
fi


# ================================================
# NODO DI DETECTION E PERCEPTION (Camera Reale o Simulata)
# ================================================

if [ "$launch_vision_node" == "true" ]; then
    echo "Avvio Detection e Perception..."
    gnome-terminal --tab --title="VisionNode" -- bash -c \
                    "source ${INSTALL_SETUP_BASH} && \
                    ros2 launch real_camera vision.launch.py; \
                    exec bash"
    sleep 1
fi


# ================================================
# TOOL DI VISUALIZZAZIONE
# ================================================

if [ "$start_image_view" == "true" ]; then
    echo "Avvio image_view..."
    gnome-terminal --tab --title="image_view" -- bash -c \
                           "source ${INSTALL_SETUP_BASH} && \
                           ros2 run image_view image_view --ros-args -r \
                           image:=/camera/camera/color/image_raw; \
                           exec bash"
    sleep 1
fi

if [ "$start_Rviz" == "true" ]; then
    echo "Avvio Rviz..."
    gnome-terminal --tab --title="Rviz" -- bash -c "source ${INSTALL_SETUP_BASH} && \
                           ros2 launch real_camera test_vision_Rviz.launch.py; \
                           exec bash"
    sleep 5
fi


# ================================================
# SCENE BUILDER
# ================================================

if [ "$launch_scene_builder" == "true" ]; then
    echo "Avvio Move Group e Scene Builder..."
    gnome-terminal --tab --title="Move Group" -- bash -c "source ${INSTALL_SETUP_BASH} && \
                           ros2 launch moveit_config move_group.launch.py; \
                           exec bash"
    sleep 2

    gnome-terminal --tab --title="Scene Builder" -- bash -c "source ${INSTALL_SETUP_BASH} && \
                           ros2 run scene_description scene_builder; \
                           exec bash"
    sleep 2
fi

# Stampa il messaggio di successo PRIMA di entrare nei cicli bloccanti
echo "Tutti i terminali e i nodi sono stati avviati!"


# ================================================
# CICLI DI AGGIORNAMENTO SCENA (Bloccanti)
# ================================================
if [ "$launch_scene_builder" == "true" ]; then

    # Uso elif per renderli mutuamente esclusivi
    if [ "$auto_loop_build_scene" == "true" ]; then
        echo "=== Aggiornamento automatico in loop della scena ==="
        echo "Aggiornamento ogni $time_between_scene_builds secondi (Premi Ctrl+C in questo terminale per fermare)."
        echo "======================================="
        while true; do 
            ros2 service call /build_scene std_srvs/srv/Trigger "{}"
            sleep $time_between_scene_builds
        done

    elif [ "$user_input_to_build_scene" == "true" ]; then
        echo "=== Controllo manuale Scene Builder ==="
        echo "Premi [INVIO] per aggiornare la scena."
        echo "Digita 'q' e premi [INVIO] per uscire (oppure usa Ctrl+C)."
        echo "======================================="

        while true; do
            read -p "Premi [INVIO] per lanciare /build_scene... " input
            if [[ "$input" == "q" || "$input" == "Q" ]]; then
                echo "Uscita dallo script."
                break
            fi
            echo "Richiamo il servizio /build_scene..."
            ros2 service call /build_scene std_srvs/srv/Trigger "{}"
            echo "---------------------------------------"
        done
    fi

fi