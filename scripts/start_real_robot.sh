#!/bin/bash


# PARAMETRI DI PERSONALIZZAZIONE ESECUZIONE OFF-LINE

#simulazione
billiard_position="cs"                         #s = sinistra, cs = centro-sinistra (TODO: c = centro, e = else da definire)

#esecuzione tiro
execute_shot="true"                            #false se vuoi solo fare visualizzazione della scena e non eseguire il tiro (utile in fase di debug e setup)
use_real_game_engine="true"                   #true se vuoi usare il game engine reale, false se vuoi usare quello fake

#logging brutal vs only essential
brutal_logging="false"                                        #true se vuoi fare logging di tutti i dati, false se vuoi fare logging solo dei dati essenziali
brutal_logging_folder="brutal_logging"                        #nome della cartella di logging, che verrà creata in data/bagdata/<logging_folder_title>
only_essential_logging_folder="only_essential_logging"        #nome della cartella di logging, che verrà creata in data/bagdata/<logging_folder_title>


# Verifica preliminare della cartella di lavoro
if [ ! -d "ws_ur5e_ballpool/install" ]; then
    echo "Errore: Cartella 'install' non trovata!"
    echo "Assicurati di lanciare questo script dalla root del workspace ROS 2."
    exit 1
fi


echo "========================================"
echo "      CONFIGURAZIONE AVVIO ROS 2        "
echo "========================================"
read -p "Avviare simulatore URSim? (s/n): " scelta_URSim


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

# ======================================================================
# PRIMO BLOCCO: Attesa e avvio del driver
# ======================================================================
SYNC_PIPE_1="/tmp/ur5e_sync_1_$$"
mkfifo "$SYNC_PIPE_1"

MSG_SYNC_1="ATTENZIONE: ATTESA DI SINCRONIZZAZIONE CON L'UTENTE (1/2)\n\nSei davvero pronto a lanciare il programma? Assicurati di aver preparato adeguatamente l'UR5e/Simulatore URSim all'inizio dell'esecuzione.\n\nPremi un tasto per avviare il DRIVER."

gnome-terminal --tab --title="Avvio esecuzione - security shell 1" -- bash -c "echo -e \"$MSG_SYNC_1\"; read -n 1 -s -r; echo ok > $SYNC_PIPE_1; exec bash"

# Mettiamo in pausa in attesa del segnale (ok)
read -r < "$SYNC_PIPE_1"
rm "$SYNC_PIPE_1"

echo "Primo segnale ricevuto! Procedo con l'avvio del driver UR5e..."
gnome-terminal --tab --title="Driver UR5e" -- bash -c "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 launch_rviz:=false; exec bash"
sleep 10


# ======================================================================
# SECONDO BLOCCO: Attesa e avvio della prossima istruzione
# ======================================================================
SYNC_PIPE_2="/tmp/ur5e_sync_2_$$"
mkfifo "$SYNC_PIPE_2"

MSG_SYNC_2="ATTENZIONE: ATTESA DI SINCRONIZZAZIONE CON L'UTENTE (2/2)\n\nVerifica che il driver sia partito correttamente.\n\nPremi un tasto per avviare la PROSSIMA ISTRUZIONE."

gnome-terminal --tab --title="Avvio esecuzione - security shell 2" -- bash -c "echo -e \"$MSG_SYNC_2\"; read -n 1 -s -r; echo ok > $SYNC_PIPE_2; exec bash"

# Mettiamo in pausa in attesa del secondo segnale (ok)
read -r < "$SYNC_PIPE_2"
rm "$SYNC_PIPE_2"

echo "Secondo segnale ricevuto! Procedo con l'istruzione successiva..."


# avvio MOVEIT 
echo "Avvio MoveIt con RViz..."
echo -e "ATTENZIONE:"
echo -e "-> Assicurati di aver decommentato il plugin FakeHardware nel file left_arm_ur5e.ros2_control.xacro sezione hardware...\n"
echo -e "-> Assicurati di aver impostato il parametro 'using_mujoco_simulation' su false nel file di configurazione task_params.yaml...\n"

sleep 2

gnome-terminal --tab --title="MoveIt+Rviz" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch moveit_config demo.launch.py; exec bash"
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

gnome-terminal --tab --title="Fake Camera" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch fake_camera fake_camera.launch.py yaml_path:=ws_ur5e_ballpool/src/camera_perception/fake_camera/config/${config_file}; exec bash"
sleep 2


# building scena RViz
echo "Avvio Scene Builder..."
gnome-terminal --tab --title="Scene Builder" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run scene_description scene_builder; exec bash"

sleep 2



if [[ "$execute_shot" == "true" ]]; then

    # attivo il cartesian pose publisher, che pubblica la posa del TCP del robot, utile per il logging e per il debug
    echo "Avvio Cartesian Pose  Publisher..."
    gnome-terminal --tab --title="Cartesian Pose Publisher" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch logging_nodes cartesian_pub_launcher.launch.py; exec bash"

    sleep 2


    #se logging brutale, avvio solo il nodo cartesian, ma non il bag writer, perché farò un ros2 bag record manuale che registri tutti i topic in un altro terminale
    if [[ "$brutal_logging" == "true" ]]; then
    
        echo "Avvio ros2 bag record manuale..."
        rm -rf "data/bagdata/${brutal_logging_folder}"
        gnome-terminal --tab --title="ros2 bag record" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 bag record -o data/bagdata/${brutal_logging_folder} -a; exec bash"

        sleep 2

    else
        echo "Avvio Bag Writer essenziale su richiesta..."
       
        gnome-terminal --tab --title="Logging nodes" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch logging_nodes bag_writer.launch.py test_title:='${only_essential_logging_folder}' ; exec bash"
        sleep 2
    fi

    

    # nodo che effettua il tiro vero e proprio
    echo "Avvio Shot Planning..."
    gnome-terminal --tab --title="Shot Planning" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning task_node --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/execution_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/shot_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/planning_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/logging_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/moveit_fix.yaml; exec bash"
    sleep 5


    # game engine: se la variabile è vera, avvio il game engine reale, altrimenti quello fake
    if [[ "$use_real_game_engine" == "true" ]]; then
        
        echo "Avvio Game Engine Reale..."
        gnome-terminal --tab --title="Game Engine Reale" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning game_engine --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/game_engine_params.yaml; exec bash"
    else
        echo "Avvio Fake Game Engine..."
        gnome-terminal --tab --title="Fake Game Engine" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning fake_game_engine --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/game_engine_params.yaml; exec bash"
        
    fi

fi


echo "Tutti i nodi sono stati avviati!"

echo "Tutti i nodi sono stati avviati!"