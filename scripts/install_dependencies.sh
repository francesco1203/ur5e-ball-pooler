#!/bin/bash


#ATTENZIONE: 
# 1. Questo script è destinato a essere eseguito su un sistema Ubuntu con ROS 2 Jazzy installato. Assicurati di avere i permessi necessari per eseguire comandi con sudo.
# 2. Verranno eseguiti comandi che richiedono privilegi di amministratore, quindi ti verrà chiesto di inserire la password dell'utente.
# 3. Lo script installerà pacchetti ROS 2, librerie e dipendenze Python necessarie per il progetto. Assicurati di avere una connessione Internet attiva durante l'esecuzione dello script.
# 4. Lo script installerà pacchetti Python a livello globale, il che potrebbe influenzare altri progetti Python sul sistema. Se preferisci un ambiente isolato, considera l'uso di virtualenv o conda.
# 5. Non si assume alcuna responsabilità per eventuali problemi derivanti dall'esecuzione di questo script. Esegui a tuo rischio e pericolo.


# SCEGLI LE OPZIONI DI INSTALLAZIONE A SECONDA DI CIÒ CHE VUOI ESEGUIRE

# both simulated and real robot ros2 environment
install_moveit=true
install_ur_description=true

# only for mujoco simulated robot in ros2 environment
install_mujoco_plugin=true

#only for using real robot in ros2 environment
install_ur_driver=true

#only for using realsense camera in ros2 environment
install_realsense_camera=true

#only for data analysis
install_data_analysis=true

#only for camera calibration in ros2 project
install_camera_calibration_dependencies=true

#only for tool calibration in ros2 project
install_ceres_optimization=true

#only for image_view in ros2 project
install_image_view=true



# Interrompe lo script se un comando fallisce
set -e

echo "========================================="
echo "🔹 Installazione Dipendenze Progetto 🔹"
echo "========================================="


# aggiorna la lista dei pacchetti e installa pip per python3 se non è già installato
echo -e "\nAggiornamento della lista dei pacchetti (apt update)..."
sudo apt update

echo -e "\nInstallazione dipendenze Python..."
# Verifica se pip è installato, altrimenti lo installa
if ! command -v pip &> /dev/null; then
    echo "pip non trovato. Installazione di python3-pip in corso..."
    sudo apt install -y python3-pip
fi


# INSTALLAZIONE PER BLOCCHI DEL PROGETTO ROS 2 JAZZY


# 1.
if [ "$install_moveit" = true ]; then
    echo -e "\nInstallazione pacchetti MoveIt2 e ros-control..."
    sudo apt install -y ros-jazzy-moveit
    sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros2-controllers ros-jazzy-moveit-ros-control-interface
fi

if [ "$install_ur_description" = true ]; then
    echo -e "\nInstallazione pacchetti UR Description..."
    sudo apt install -y ros-jazzy-ur-description
fi


# 2.
if [ "$install_mujoco_plugin" = true ]; then
    echo -e "\nInstallazione pacchetto plug-in MuJoCo ROS2 Control..."
    sudo apt install -y ros-jazzy-mujoco-ros2-control
    sudo apt install -y ros-jazzy-mujoco-ros2-control-plugins
    sudo apt install -y ros-jazzy-mujoco-vendor
fi 


# 3.
if [ "$install_ur_driver" = true ]; then
    echo -e "\nInstallazione pacchetti UR Driver..."
    sudo apt install -y ros-jazzy-ur-robot-driver
    sudo apt install -y ros-jazzy-ur
fi  


# 4. 
if [ "$install_realsense_camera" = true ]; then
    echo -e "\nInstallazione pacchetti Intel Realsense Camera..."
    sudo apt install -y ros-jazzy-realsense2-camera
    sudo apt install -y ros-jazzy-realsense2-description
    sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-vision-opencv libopencv-dev  #detection and image processing
fi

# 5.
if [ "$install_data_analysis" = true ]; then
    echo -e "\nInstallazione pacchetti Python per l'analisi dati..."
    pip install rosbags pandas numpy scipy matplotlib --break-system-packages
fi

# 6.
if [ "$install_camera_calibration_dependencies" = true ]; then
    echo -e "\nInstallazione pacchetti per la calibrazione della camera..."
    pip install ur_rtde tqdm termcolor --break-system-packages
fi


#7.
if [ "$install_ceres_optimization" = true ]; then
    echo -e "\nInstallazione pacchetti per la calibrazione end effector..."
    sudo apt install -y libceres-dev
fi

#8.
if [ "$install_image_view" = true ]; then
    echo -e "\nInstallazione image view..."
    sudo apt install -y ros-jazzy-image-view
fi


echo "========================================="
echo "✅ Tutte le dipendenze sono state installate con successo!"
echo "========================================="