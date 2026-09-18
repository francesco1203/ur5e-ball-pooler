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
#------------------------------------------------


# ------------------------------------------------
# PARAMETRI DI PERSONALIZZAZIONE ESECUZIONE OFF-LINE

#launch driver (se robot/container è già avviato e il driver è stato già lanciato, non serve rilanciare tutto)
launch_robot_driver="true"                         

#avvio camera IntelRealsense

start_image_view="false"       #simulatore per la percezione             


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
# AGGIORNAMENTO PLUGIN UR_ROBOT_DRIVER PER MOVEIT
MOVEIT_CONFIG_DIR="${WS_DIR}/src/moveit_config/config"

echo "Aggiornamento del plugin ur_robot_driver nel file ur5e.ros2_control.xacro..."
python3 ${MOVEIT_CONFIG_DIR}/ros2_control_hardware_auto_switch.py driver --robot_ip="${LAB_ROBOT_IP}" --reverse_ip=""
#------------------------------------------------


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


    #robot reale
    gnome-terminal --tab --title="Driver UR5e" -- bash -c "source \"${INSTALL_SETUP_BASH}\" && \
        ros2 launch ur_robot_driver ur_control.launch.py \
            ur_type:=ur5e \
            robot_ip:=${LAB_ROBOT_IP} \
            launch_rviz:=false \
            description_file:=${ARM_DESC_DIR}/urdf/arm_driver_wrapper.urdf.xacro \
            kinematics_params_file:=${ARM_DESC_DIR}/config/uclv_right_ur5e_kinematics.yaml; \
        exec bash"
    
else
    echo -e "\nDriver UR5e non avviati da questo script. Assicurati che siano già attivi..."
fi



# ================================================
# AVVIO RVIZ
# ================================================
echo -e "\n================================================================="
echo -e "ATTENZIONE: ATTESA DI SINCRONIZZAZIONE CON L'UTENTE"
echo -e "================================================================="
echo -e "Verifica che il driver sia partito correttamente.\nDevi aver selezionato \"External control\" dal teach pendant e aver premuto Play.\nDal terminale del driver dovresti vedere la scritta 'Robot ready to receive commands'.\n"
echo -e "Premi un tasto per avviare Rviz..."

# Mettiamo in pausa in attesa del segnale
read -n 1 -s -r

# echo "Avvio MoveIt..."
# gnome-terminal --tab --title="MoveGroup" -- bash -c \
#                 "source ${INSTALL_SETUP_BASH} && \
#                 ros2 launch moveit_config move_group.launch.py \
#                     use_sim_time:=false; \
#                 exec bash"
# sleep 5


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

#avvio driver della camera
echo "Avvio driver IntelRealSense..."
gnome-terminal --tab --title="Intel Realsense Camera" -- bash -c \
                "source ${INSTALL_SETUP_BASH} && \
                ros2 launch realsense2_camera rs_launch.py \
                    depth_module.depth_profile:=1280x720x30 \
                    pointcloud.enable:=true \
                    enable_rgbd:=true \
                    enable_sync:=true \
                    align_depth.enable:=true ; \
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


# ================================================
# BUILDER DELLA SCENA
# ================================================
echo "Avvio Scene Builder..."
gnome-terminal --tab --title="Scene Builder" -- bash -c \
                "source ${INSTALL_SETUP_BASH} && \
                ros2 run scene_description scene_builder; \
                exec bash"

sleep 2

echo "Tutti i nodi sono stati avviati!"