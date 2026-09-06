#!/bin/bash

cd ws_ur5e_ballpool || { echo "Errore: Impossibile entrare nella cartella ws_ur5e_ballpool"; exit 1; }
source /opt/ros/jazzy/setup.bash || { echo "Errore: Impossibile eseguire source /opt/ros/jazzy/setup.bash"; exit 1; }
colcon build --symlink-install || { echo "Errore: Impossibile eseguire colcon build"; exit 1; }

