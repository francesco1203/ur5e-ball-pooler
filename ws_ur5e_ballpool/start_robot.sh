#!/bin/bash

# Verifica preliminare della cartella di lavoro
if [ ! -d "install" ]; then
    echo "Errore: Cartella 'install' non trovata!"
    echo "Assicurati di lanciare questo script dalla root del workspace ROS 2."
    exit 1
fi

# Parametri di personalizzazione esecuzione off-line
build_scene_rviz="true"
execute_shot="true"             #false se vuoi solo fare visualizzazione della scena e non eseguire il tiro
logging_enabled="true"
use_real_game_engine="true"


echo "========================================"
echo "      CONFIGURAZIONE AVVIO ROS 2        "
echo "========================================"
read -p "Vuoi usare MuJoCo? (s/n): " scelta_mujoco


# Gestione della scelta con if-else
if [[ "$scelta_mujoco" =~ ^[sS][iI]?$ ]]; then

    # devo generare la scena completa con le palline per MuJoCo con lo script autocreate_complete_scene.py
    python3 ./src/camera_perception/fake_camera/mujoco_automation/autocreate_complete_scene.py

    # Controlliamo se lo script python è andato a buon fine ($? == 0)
    if [ $? -ne 0 ]; then
        echo "ERRORE: Generazione della scena MuJoCo fallita! Interruzione."
        exit 1
    fi
    echo -e "Scena MuJoCo generata con successo!\n"


    #avvio del simulatore vero e proprio con MuJoCo e MoveIt
    echo "Avvio MuJoCo con MoveIt..."
    echo -e "ATTENZIONE:"
    echo -e "-> Assicurati di aver decommentato il plugin MuJoCo nel file left_arm_ur5e.ros2_control.xacro sezione hardware...\n"
    echo -e "-> Assicurati di aver impostato il parametro 'using_mujoco_simulation' su true nel file di configurazione task_params.yaml...\n"
    
    sleep 2

    read -p "Vuoi avviare anche RViz? (s/n): " scelta_rviz
    if [[ "$scelta_rviz" =~ ^[sS][iI]?$ ]]; then
        gnome-terminal --tab --title="Moveit+MuJoCo+Rviz" -- bash -c "source install/setup.bash && ros2 launch moveit_config mujoco_and_rviz_demo.launch.py; exec bash"
        sleep 10
    else
        gnome-terminal --tab --title="Moveit+MuJoCo" -- bash -c "source install/setup.bash && ros2 launch moveit_config mujoco_demo.launch.py; exec bash"
        sleep 5
    fi

else
    echo "Avvio MoveIt con RViz..."
    echo -e "ATTENZIONE:"
    echo -e "-> Assicurati di aver decommentato il plugin FakeHardware nel file left_arm_ur5e.ros2_control.xacro sezione hardware...\n"
    echo -e "-> Assicurati di aver impostato il parametro 'using_mujoco_simulation' su false nel file di configurazione task_params.yaml...\n"
    
    sleep 2

    gnome-terminal --tab --title="MoveIt+Rviz" -- bash -c "source install/setup.bash && ros2 launch moveit_config demo.launch.py; exec bash"
    sleep 10
fi



echo "Avvio Fake Camera..."
gnome-terminal --tab --title="Fake Camera" -- bash -c "source install/setup.bash && ros2 launch fake_camera fake_camera.launch.py; exec bash"

sleep 2


# building scena RViz
if [[ "$build_scene_rviz" == "true" ]]; then
    echo "Avvio Scene Builder..."
    gnome-terminal --tab --title="Scene Builder" -- bash -c "source install/setup.bash && ros2 run scene_description scene_builder; exec bash"

    sleep 2
fi



if [[ "$execute_shot" == "true" ]]; then

    # logging dei dati di shot planning
    if [[ "$logging_enabled" == "true" ]]; then
        echo "Avvio Nodi logger..."
        gnome-terminal --tab --title="Nodi di logging" -- bash -c "source install/setup.bash && ros2 launch execution_monitoring loggers.launch.py; exec bash"

        sleep 2
    fi

    #tiro vero e proprio
    echo "Avvio Shot Planning..."
    gnome-terminal --tab --title="Shot Planning" -- bash -c "source install/setup.bash && ros2 run shot_planning task_node --ros-args --params-file src/shot_planning/config/task_params.yaml; exec bash"

    sleep 5


    # Game engine: se la variabile è vera, avvio il game engine reale, altrimenti quello fake
    if [[ "$use_real_game_engine" == "true" ]]; then
        
        echo "Avvio Game Engine Reale..."
        gnome-terminal --tab --title="Game Engine Reale" -- bash -c "source install/setup.bash && ros2 run shot_planning game_engine; exec bash"
    else
        echo "Avvio Fake Game Engine..."
        gnome-terminal --tab --title="Fake Game Engine" -- bash -c "source install/setup.bash && ros2 run shot_planning fake_game_engine --ros-args --params-file src/shot_planning/config/task_params.yaml; exec bash"
        
    fi

fi


echo "Tutti i nodi sono stati avviati!"