#!/bin/bash


# PARAMETRI DI PERSONALIZZAZIONE ESECUZIONE OFF-LINE

#avvio camera IntelRealsense
use_real_camera="false"                      #true se vuoi usare la camera reale, false se vuoi usare la camera fake (utile in fase di debug e setup)

#simulazione per camera finta
billiard_position="s"                         #s = sinistra, cs = centro-sinistra (TODO: c = centro, e = else da definire)

#esecuzione tiro
execute_shot="true"                           #false se vuoi solo fare visualizzazione della scena e non eseguire il tiro (utile in fase di debug e setup)
use_real_game_engine="true"                   #true se vuoi usare il game engine reale, false se vuoi usare quello fake

#logging:  brutal (tutti i topic) vs only_essential (solo i topic essenziali)
logging_enable="true"                                         #true se vuoi fare logging
logging_type_brutal_on="false"                                #true se vuoi fare logging brutale (di tutti i topic), false se vuoi fare logging solo dei dati essenziali
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


# avvio camera - reale: lancia i driver per la camera reale o il nodo per la fake)
if [[ "$use_real_camera" == "true" ]]; then
    echo "Avvio camera reale Intel Realsense..."
    gnome-terminal --tab --title="Intel Realsense Camera" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch realsense2_camera rs_launch.py; exec bash"
    sleep 5

    #nodo per vision..
else
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
fi



# Gestione della scelta simulatore o robot vero con if-else
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



# Avvio Driver
echo -e "\n================================================================="
echo -e "ATTENZIONE: ATTESA DI SINCRONIZZAZIONE CON L'UTENTE"
echo -e "================================================================="
echo -e "Verifica che il robot (simulato o reale) sia pronto per l'operatività.\nDevi averlo acceso e aver selezionato senza problemi external control nel pannello di controllo.\n"
echo -e "Premi un tasto per avviare i driver..."

# Mettiamo in pausa in attesa del segnale
read -n 1 -s -r

echo -e "\nProcedo con l'avvio del driver UR5e..."
gnome-terminal --tab --title="Driver UR5e" -- bash -c "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 launch_rviz:=false; exec bash"


# Avvio MoveIt
echo -e "\n================================================================="
echo -e "ATTENZIONE: ATTESA DI SINCRONIZZAZIONE CON L'UTENTE"
echo -e "================================================================="
echo -e "Verifica che il driver sia partito correttamente.\nDevi aver selezionato senza problemi external control nel pannello di controllo dell'UR5e.\n"
echo -e "Premi un tasto per avviare Moveit..."

# Mettiamo in pausa in attesa del segnale
read -n 1 -s -r

echo "Avvio MoveIt con RViz..."
echo -e "ATTENZIONE:"
echo -e "-> Assicurati di aver decommentato il plugin FakeHardware nel file left_arm_ur5e.ros2_control.xacro sezione hardware..."

sleep 2

gnome-terminal --tab --title="MoveIt+Rviz" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch moveit_config demo.launch.py; exec bash"
sleep 10


# building scena RViz
echo "Avvio Scene Builder..."
gnome-terminal --tab --title="Scene Builder" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run scene_description scene_builder; exec bash"

sleep 2



if [[ "$execute_shot" == "true" ]]; then

    # gestione del logging
    if [["$logging_enabled" == "true" ]]; then
        echo "Avvio Logging Nodes..."
        gnome-terminal --tab --title="Logging nodes" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch logging_nodes logging_nodes.launch.py; exec bash"
        sleep 2

        #se logging brutale, non avvio il bag writer, perché farò un ros2 bag record manuale che registri tutti i topic in un altro terminale
        if [[ "$brutal_logging" == "true" ]]; then
        
            echo "Avvio ros2 bag record manuale..."
            rm -rf "data/bagdata/${brutal_logging_folder}"  #cancello la cartella di logging precedente se esiste, così da non avere conflitti
            gnome-terminal --tab --title="ros2 bag record" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 bag record -o data/bagdata/${brutal_logging_folder} -a; exec bash"

            sleep 2

        else
            echo "Avvio Bag Writer essenziale su richiesta..."
        
            gnome-terminal --tab --title="Logging nodes" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch logging_nodes bag_writer.launch.py test_title:='${only_essential_logging_folder}' ; exec bash"
            sleep 2
        fi
    fi


    # nodo che effettua il tiro vero e proprio
    if [["$logging_enabled" == "true" ]]; then      
            # Con logging
            echo "Avvio Shot Planning con setup Logging..."      
            gnome-terminal --tab --title="Shot Planning" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning task_node --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/execution_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/shot_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/planning_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/logging_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/moveit_fix.yaml; exec bash"
        else
            # Senza  logging
            echo "Avvio Shot Planning senza logging..."
            gnome-terminal --tab --title="Shot Planning" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning task_node --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/execution_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/shot_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/planning_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/moveit_fix.yaml; exec bash"
        fi


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