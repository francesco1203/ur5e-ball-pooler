#!/bin/bash

# Verifica preliminare della cartella di lavoro
if [ ! -d "install" ]; then
    echo "Errore: Cartella 'install' non trovata!"
    echo "Assicurati di lanciare questo script dalla root del workspace ROS 2."
    exit 1
fi

# echo "========================================"
# echo "      CONFIGURAZIONE AVVIO ROS 2        "
# echo "========================================"
# read -p "Vuoi avviare MuJoCo? (s/n): " scelta_mujoco

# # Gestione della scelta con if-else
# if [[ "$scelta_mujoco" =~ ^[sS][iI]?$ ]]; then

#     echo "Avvio MuJoCo con MoveIt..."
#     echo "-> Assicurati di aver decommentato il plugin MuJoCo nel file left_arm_ur5e.ros2_control.xacro sezione hardware..."
#     gnome-terminal --tab --title="Moveit+MuJoCo" -- bash -c "source install/setup.bash && ros2 launch moveit_config mujoco_demo.launch.py; exec bash"

#     sleep 10
# else
#     echo "Avvio MoveIt..."
#     echo "-> Assicurati di aver decommentato il plugin FakeHardware nel file left_arm_ur5e.ros2_control.xacro sezione hardware..."
#     gnome-terminal --tab --title="MoveIt" -- bash -c "source install/setup.bash && ros2 launch moveit_config demo.launch.py; exec bash"

#     sleep 10

# fi


echo "Avvio MoveIt..."
echo "-> Assicurati di aver decommentato il plugin FakeHardware nel file left_arm_ur5e.ros2_control.xacro sezione hardware..."
gnome-terminal --tab --title="MoveIt" -- bash -c "source install/setup.bash && ros2 launch moveit_config demo.launch.py; exec bash"

sleep 10


echo "Avvio Fake Camera..."
gnome-terminal --tab --title="Fake Camera" -- bash -c "source install/setup.bash && ros2 launch fake_camera fake_camera.launch.py; exec bash"

sleep 3

echo "Avvio Scene Builder..."
gnome-terminal --tab --title="Scene Builder" -- bash -c "source install/setup.bash && ros2 run scene_description scene_builder; exec bash"

sleep 7

echo "Avvio Shot Planning..."
gnome-terminal --tab --title="Shot Planning" -- bash -c "source install/setup.bash && ros2 run shot_planning task_node --ros-args --params-file src/shot_planning/config/task_params.yaml; exec bash"

echo "Tutti i nodi sono stati avviati!"