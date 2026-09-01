#!/bin/bash

# Verifica preliminare della cartella di lavoro
if [ ! -d "install" ]; then
    echo "Errore: Cartella 'install' non trovata!"
    echo "Assicurati di lanciare questo script dalla root del workspace ROS 2."
    exit 1
fi

# Parametri di personalizzazione esecuzione off-line
billiard_position="cs"                    #s = sinistra, cs = centro-sinistra (TODO: c = centro, e = else da definire)
build_scene_rviz="true"
execute_shot="true"                      #false se vuoi solo fare visualizzazione della scena e non eseguire il tiro
logging_enabled="true"
use_real_game_engine="true"              #true se vuoi usare il game engine reale, false se vuoi usare quello fake


echo "========================================"
echo "      CONFIGURAZIONE AVVIO ROS 2        "
echo "========================================"
read -p "Avviare URSim? (s/n): " scelta_URSim


# Gestione della scelta con if-else
if [[ "$scelta_URSim" =~ ^[sS][iI]?$ ]]; then

    echo -e "Avvio URSim in un container Docker...\n"


    # Forza la chiusura e la rimozione di un eventuale container precedente
    echo "Pulizia di vecchi container URSim in corso..."
    docker rm -f ursim >/dev/null 2>&1 || true


    echo -e "->ATTENZIONE: assicurati di aver installato Docker. Al primo avvio potrebbe essere necessario creare il container e richiede alcuni minuti...\n"
    echo -e "->Il container sarà raggiungibile all'indirizzo indicato nel terminale..\n"

    gnome-terminal --tab --title="URSim Launcher" -- bash -c "ros2 run ur_client_library start_ursim.sh; exec bash"
    sleep 5

else
    echo "Avvio senza URSim..."
    sleep 2

fi


# Attesa di sincronizzazione con l'utente prima di procedere con l'apertura delle altre finestre
# SICUREZZA: questo meccanismo di sincronizzazione è necessario per evitare che l'utente non abbia ancora preparato l'UR5e all'inizio dell'esecuzione, e quindi il programma potrebbe partire senza che l'UR5e sia pronto, causando errori.
SYNC_PIPE="/tmp/ur5e_sync_$$"
mkfifo "$SYNC_PIPE"

# Salviamo il messaggio in una variabile esterna per evitare conflitti con gli apostrofi (apici singoli)
MSG_SYNC="ATTENZIONE: ATTESA DI SINCRONIZZAZIONE CON L'UTENTE\n\nSei davvero pronto a lanciare il programma? Assicurati di aver preparato adeguatamente l'UR5e/Simulatore URSim all'inizio dell'esecuzione.\n\nPremi un tasto per continuare solo quando sei pronto."

# Passiamo la variabile espansa, senza usare apici singoli internamente
gnome-terminal --tab --title="Avvio esecuzione - security shell" -- bash -c "echo -e \"$MSG_SYNC\"; read -n 1 -s -r; echo ok > $SYNC_PIPE; exec bash"

# Mettiamo in pausa in attesa del segnale (ok)
read -r < "$SYNC_PIPE"
rm "$SYNC_PIPE"

echo "Segnale ricevuto! Procedo con l'apertura delle altre finestre..."


# avvio del driver
echo "Avvio driver UR5e..."
gnome-terminal --tab --title="Driver UR5e" -- bash -c "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 launch_rviz:=false; exec bash"
sleep 2



# avvio MOVEIT 
echo "Avvio MoveIt con RViz..."
echo -e "ATTENZIONE:"
echo -e "-> Assicurati di aver decommentato il plugin FakeHardware nel file left_arm_ur5e.ros2_control.xacro sezione hardware...\n"
echo -e "-> Assicurati di aver impostato il parametro 'using_mujoco_simulation' su false nel file di configurazione task_params.yaml...\n"

sleep 2

gnome-terminal --tab --title="MoveIt+Rviz" -- bash -c "source install/setup.bash && ros2 launch moveit_config demo.launch.py; exec bash"
sleep 10


# avvio fake camera con la posizione scelta
case "$billiard_position" in
    "s")
        echo "Avvio Fake Camera (tavolo a sinistra)..."
        config_file="fake_camera_config_sx.yaml"
        ;;
    "cs")
        echo "Avvio Fake Camera (tavolo al centro-sinistra)..."
        config_file="fake_camera_config_center_sx.yaml"
        ;;
    *)
        # Caso di default se la variabile non corrisponde a nessuno dei precedenti
        echo "Errore: posizione '$billiard_position' non riconosciuta."
        exit 1
        ;;
esac

gnome-terminal --tab --title="Fake Camera" -- bash -c "source install/setup.bash && ros2 launch fake_camera fake_camera.launch.py yaml_path:=src/camera_perception/fake_camera/config/${config_file}; exec bash"
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
        gnome-terminal --tab --title="Game Engine Reale" -- bash -c "source install/setup.bash && ros2 run shot_planning game_engine --ros-args --params-file src/shot_planning/config/game_engine_params.yaml; exec bash"
    else
        echo "Avvio Fake Game Engine..."
        gnome-terminal --tab --title="Fake Game Engine" -- bash -c "source install/setup.bash && ros2 run shot_planning fake_game_engine --ros-args --params-file src/shot_planning/config/game_engine_params.yaml; exec bash"
        
    fi

fi


echo "Tutti i nodi sono stati avviati!"