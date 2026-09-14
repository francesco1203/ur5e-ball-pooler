#!/bin/bash



# ------------------------------------------------
# PARAMETRI DI PERSONALIZZAZIONE ESECUZIONE OFF-LINE

#true se vuoi usare la camera reale
use_real_camera="false"                      #true se vuoi usare la camera reale, false se vuoi usare i bag files o il nodo fake_camera_node
launch_driver="false"                         #true se vuoi (ri)lanciare il driver (avviato solo con camera reale)


#strumenti simulati (se non si usa la camera reale)
use_bagfiles_stream="false"                     #true se vuoi usare i bag files per simulare lo stream della camera
bagfiles_path="data/bagdata/camera_stream/.."   #percorso del file bag da usare se use_bagfiles_stream=true

use_mujoco_camera="true"                        #true se vuoi usare la camera simulata in MuJoCo (per testare la percezione in tempo reale)

use_fake_camera_node="false"                    #true se vuoi usare il nodo fake_camera_node per simulare completamente la detection a valle


#parte vision
launch_vision_node="false"                   #true se vuoi lanciare il nodo vision_node (per testare la percezione in tempo reale)


#simulatori per la percezione
start_Rviz="false"                                 #true se vuoi lanciare Rviz per visualizzare la scena e i risultati della percezione
start_rqt_image_view="true"                        #true se vuoi lanciare rqt_image_view per visualizzare i topic della camera (color, depth, info)


#costruzione scena
launch_scene_builder="false"                  #true se vuoi lanciare il nodo scene_builder (per testare la percezione in tempo reale)
auto_loop_build_scene="true"                   #true se vuoi che la scena venga costruita in loop (per testare la percezione in tempo reale)
time_between_scene_builds=1                    #tempo in secondi tra una costruzione della scena e la successiva (se auto_loop_build_scene=true)
user_input_to_build_scene="true"               #true se vuoi che la costruzione della scena avvenga solo dopo un input dell'utente (se auto_loop_build_scene=true)
# ------------------------------------------------


# ------------------------------------------------
# Verifica preliminare della cartella di lavoro
if [ ! -d "ws_ur5e_ballpool/install" ]; then
    echo "Errore: Cartella 'install' non trovata!"
    echo "Assicurati di lanciare questo script dalla root del workspace ROS 2."
    exit 1
fi
# ------------------------------------------------



if [ "$use_real_camera" = true ] && [ "$launch_driver" = true ]; then

    # ------------------------------------------------
    # avvio driver per la camera reale per stream live

    if [ "$launch_driver" = true ]; then
        echo "Avvio driver IntelRealSense..."
        gnome-terminal --tab --title="Intel Realsense Camera" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch realsense2_camera rs_launch.py; exec bash"
        sleep 5
    else
        echo -e "Driver IntelRealSense non avviati da questo script. Assicurati che siano già attivi..."
    fi
    # ------------------------------------------------


    # ------------------------------------------------
    # avvio del nodo di detection e perception che lavora con la camera reale

    if [ "$launch_vision_node" = true ]; then
        echo "Avvio Detection e Perception..."
        gnome-terminal --tab --title="VisionNode" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch real_camera vision.launch.py; exec bash"
        sleep 1
    fi
    # ------------------------------------------------

else
    #strumenti simulati

    if [ "$use_bagfiles_stream" = true ]; then
        # ------------------------------------------------
        # avvio stream da bag files

        echo "Avvio stream da bag files..."
        #...(TODO: implementare avvio stream da bag files)

        sleep 5
        # ------------------------------------------------

        # ------------------------------------------------
        # avvio del nodo di detection e perception che lavora con la camera reale

        if [ "$launch_vision_node" = true ]; then
            echo "Avvio Detection e Perception..."
            gnome-terminal --tab --title="VisionNode" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch real_camera vision.launch.py; exec bash"
            sleep 1
        fi
        # ------------------------------------------------

    fi

    if [ "$use_mujoco_camera" = true ]; then
        # ------------------------------------------------
        # avvio camera simulata in MuJoCo

        # aggiornamento del plugin di MuJoCo nel file ur5e.ros2_control.xacro
        echo "Aggiornamento del plugin MuJoCo nel file ur5e.ros2_control.xacro..."
        python3 ws_ur5e_ballpool/src/moveit_config/config/ros2_control_hardware_auto_switch.py mujoco


        # devo generare la scena completa con le palline per MuJoCo con lo script autocreate_complete_scene.py
        echo "Aggiornamento del file della scena MuJoCo con posizione delle palline..."
        python3 ws_ur5e_ballpool/src/camera_perception/fake_camera/mujoco_automation/autocreate_complete_scene.py --yaml_path ws_ur5e_ballpool/src/camera_perception/fake_camera/config/fake_camera_config.yaml
                

        # avvio MuJoCo con la scena completa
        echo "Avvio scena con camera simulata in MuJoCo..."
        gnome-terminal --tab --title="MuJoCo Simulation" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch fake_camera mujoco_with_stream.launch.py; exec bash"
        sleep 2

        # ------------------------------------------------
        # avvio del nodo di detection e perception che lavora con la camera simulata in
        if [ "$launch_vision_node" = true ]; then
            echo "Avvio Detection e Perception..."
            echo "NOTA: effettuata rimappatura dei topic da MuJoCo a quelli attesi dal nodo vision_node (color_topic, depth_topic, info_topic)"
            gnome-terminal --tab --title="VisionNode" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch real_camera vision.launch.py color_topic:=/realsense_d435/color depth_topic:=/realsense_d435/depth info_topic:=/realsense_d435/camera_info; exec bash"
            sleep 1
        fi
        # ------------------------------------------------  
    fi


    if [ "$use_fake_camera_node" = true ]; then
        # ------------------------------------------------
        # avvio fake camera

        echo "Avvio fake camera..."
        gnome-terminal --tab --title="Fake Camera" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch fake_camera fake_camera.launch.py; exec bash"
        sleep 1
        # ------------------------------------------------

        #NOTA: NON SERVE VISION NODE, PERCHE' IL NODO FAKE CAMERA SIMULA A VALLE DELLA DETECTION
    fi
    
fi


# ------------------------------------------------
# Avvio Rviz

if [ "$start_Rviz" = true ]; then
    echo "Avvio Rviz..."
    sleep 2

    gnome-terminal --tab --title="Rviz" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch real_camera test_vision_Rviz.launch.py; exec bash"
    sleep 5
fi
# ------------------------------------------------


#   ------------------------------------------------
# Avvio rqt_image_view
if [ "$start_rqt_image_view" = true ]; then
    echo "Avvio rqt_image_view..."
    sleep 2

    #gnome-terminal --tab --title="rqt_image_view" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run rqt_image_view rqt_image_view; exec bash"
    gnome-terminal --tab --title="rqt_image_view" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run rqt_image_view rqt_image_view --ros-args -p use_sim_time:=true; exec bash"
    
    sleep 5
fi
# ------------------------------------------------


if [ "$launch_scene_builder" = true ]; then
   # ------------------------------------------------
    # avvio builder della scena

    echo "Avvio Scene Builder..."   #nota: ho bisogno di movegroup, che avvio sotto subito prima

    gnome-terminal --tab --title="Move Group" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch moveit_config move_group.launch.py; exec bash"
    sleep 2

    gnome-terminal --tab --title="Scene Builder" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run scene_description scene_builder; exec bash"
    sleep 2
    # ------------------------------------------------


    # ------------------------------------------------
    # costruzione della scena in loop (per testare la percezione in tempo reale)

    if [ "$auto_loop_build_scene" = true ]; then
        echo "=== Aggiornamento automatico in loop della scena ==="
        echo "Aggiornamento ogni $time_between_scene_builds secondi."
        echo "======================================="

        while true; do ros2 service call /build_scene std_srvs/srv/Trigger "{}"; sleep $time_between_scene_builds; done
    fi

    if [ "$user_input_to_build_scene" = true ]; then
        echo "=== Controllo manuale Scene Builder ==="
        echo "Premi [INVIO] per aggiornare la scena."
        echo "Digita 'q' e premi [INVIO] per uscire (oppure usa Ctrl+C)."
        echo "======================================="

        while true; do
            # Attende l'input dell'utente
            read -p "Premi [INVIO] per lanciare /build_scene... " input
            
            # Se l'utente digita 'q' o 'Q', esce dal ciclo
            if [[ "$input" == "q" || "$input" == "Q" ]]; then
                echo "Uscita dallo script."
                break
            fi
            
            # Chiama il servizio
            echo "Richiamo il servizio /build_scene..."
            ros2 service call /build_scene std_srvs/srv/Trigger "{}"
            echo "---------------------------------------"
        done
        fi
    #------------------------------------------------
fi


echo "Tutti i nodi sono stati avviati!"