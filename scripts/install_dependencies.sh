#!/bin/bash


#ATTENZIONE: Questo script è destinato a essere eseguito su un sistema Ubuntu con ROS 2 Jazzy installato. Assicurati di avere i permessi necessari per eseguire comandi con sudo.
# FILE NON ANCORA TESTATO. UTILIZZARE CON CAUTELA.


# LISTA DELLE DIPENDENZE DA INSTALLARE:
# - ur_description (Ros2 standard package) - models and config files
# - ur (Ros2 standard package) - driver ur5
# - ros-jazzy-realsense2-camera ros-jazzy-realsense2-description (Ros2 standard package) - driver IntelRealSense
# - moveit2 (Ros2 standard package)
# - mujoco_ros2_control (Ros2 package) - plug-in bridge for mujoco simulator
# - python packs: rosbags, pandas, numpy, matplotlib, scipy - for trajectory and data analysis



# Interrompe lo script se un comando fallisce
set -e

echo "========================================="
echo "🔹 Installazione Dipendenze Progetto 🔹"
echo "========================================="

echo -e "\nAggiornamento della lista dei pacchetti (apt update)..."
sudo apt update

echo -e "\nInstallazione pacchetti standard ROS 2 Jazzy..."
# Installiamo le dipendenze UR, RealSense, MoveIt2 e MuJoCo
# Nota: L'opzione -y accetta automaticamente i prompt di installazione
sudo apt install -y \
    ros-jazzy-ur \
    ros-jazzy-ur-description \
    ros-jazzy-ur-robot-driver \
    ros-jazzy-realsense2-camera \
    ros-jazzy-realsense2-description \
    ros-jazzy-moveit \
    ros-jazzy-mujoco-ros2-control

echo -e "\nInstallazione dipendenze Python per l'analisi dati..."
# Verifica se pip è installato, altrimenti lo installa
if ! command -v pip &> /dev/null; then
    echo "pip non trovato. Installazione di python3-pip in corso..."
    sudo apt install -y python3-pip
fi

# Installazione forzata dei pacchetti Python a livello globale (come concordato in precedenza)
pip install rosbags pandas numpy scipy matplotlib --break-system-packages

echo "========================================="
echo "✅ Tutte le dipendenze sono state installate con successo!"
echo "========================================="