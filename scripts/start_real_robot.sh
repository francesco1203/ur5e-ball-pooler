#!/bin/bash

# ------------------------------------------------
# PARAMETRI DI RETE

robot_ip="192.168.56.101"       
reverse_ip="192.168.56.1"       #il tuo ip
#------------------------------------------------


# ------------------------------------------------
# PARAMETRI DI PERSONALIZZAZIONE ESECUZIONE OFF-LINE

#avvio camera IntelRealsense
use_real_camera="false"                      #true se vuoi usare la camera reale con detection, false se vuoi usare la camera fake (utile in fase di debug e setup)

#esecuzione tiro
execute_shot="true"                           #false se vuoi solo fare visualizzazione della scena e non eseguire il tiro (utile in fase di debug e setup)
use_real_game_engine="true"                   #true se vuoi usare il game engine reale, false se vuoi usare quello fake

#logging:  brutal (tutti i topic) vs only_essential (solo i topic essenziali)
logging_enable="false"                                         #true se vuoi fare logging
brutal_logging="false"                                #true se vuoi fare logging brutale (di tutti i topic), false se vuoi fare logging solo dei dati essenziali
brutal_logging_folder="brutal_logging"                        #nome della cartella di logging, che verrà creata in data/bagdata/<logging_folder_title>
only_essential_logging_folder="only_essential_logging"        #nome della cartella di logging, che verrà creata in data/bagdata/<logging_folder_title>
# ------------------------------------------------


# ------------------------------------------------
# Verifica preliminare della cartella di lavoro
if [ ! -d "ws_ur5e_ballpool/install" ]; then
    echo "Errore: Cartella 'install' non trovata!"
    echo "Assicurati di lanciare questo script dalla root del workspace ROS 2."
    exit 1
fi

source ws_ur5e_ballpool/install/setup.bash
# ------------------------------------------------


#------------------------------------------------
# Avvio e addestramento dell'utente
echo ""
echo "========================================"
echo "      CONFIGURAZIONE AVVIO ROS 2        "
echo "========================================"
read -p "Avviare in simulazione con URSim? (s/n): " scelta_URSim


if [[ "$scelta_URSim" =~ ^[sS][iI]?$ ]]; then
    echo "Hai scelto di avviare in simulazione con URSim."

    cat << 'EOF'

Sequenza corretta (Simulazione):
 
  1. Avvio Container Docker (URSim) ✅
    - Lo script lancia il container
    - Attendi che si apra l'interfaccia URSim
  2. Accensione Robot Virtuale ✅
    - Nel teach pendant di URSim: clicca "Power Off" → "Normal"
    - Attendi che il robot si inizializzi (led diventa verde)
    - Lo script aspetta che tu prema un tasto per confermare
  3. Avvio Driver ✅
    - Il driver parte e si connette al robot
    - MA il robot NON è ancora pronto a ricevere comandi!
  4. Avvio External Control Program ⚠️ PASSAGGIO CRUCIALE
    - Nel teach pendant di URSim:
        - Vai su "Program" → "URCaps" → "External Control"
      - Verifica che l'IP sia corretto (192.168.56.1)
      - Premi "Play" ▶️
    - Solo ora il robot accetta comandi dal driver
    - Lo script aspetta che tu prema un tasto dopo aver visto "Robot ready to receive commands"
  5. Avvio MoveIt ✅
    - Ora MoveIt può comandare il robot
 
Quindi: Docker → Accendi Robot → Driver → External Control → MoveIt 🎯
EOF

else
    echo "Hai scelto di avviare senza URSim (robot reale)."

    cat << 'EOF'

Sequenza corretta (Robot Reale):
 
  1. Preparazione Robot Fisico ✅
    - Accendi il controller e sblocca i giunti (rilascia i freni, led verde)
    - Assicurati che il cavo di rete sia collegato tra PC e controller
  2. Avvio Driver ✅
    - Il driver parte e si connette all'IP del robot reale
    - MA il robot NON è ancora pronto a ricevere comandi!
  3. Avvio External Control Program ⚠️ PASSAGGIO CRUCIALE
    - Sul Teach Pendant fisico del robot:
        - Apri/Crea un programma contenente il nodo "External Control"
      - Verifica in "Installation" che l'IP dell'Host (il tuo PC) sia corretto
      - Premi "Play" ▶️
    - Solo ora il robot accetta comandi dal driver
    - Lo script aspetta che tu prema un tasto dopo aver visto "Robot ready to receive commands"
  4. Avvio MoveIt ✅
    - Ora MoveIt può comandare il robot fisico
 
Quindi: Accendi Robot → Driver → External Control → MoveIt 🎯
EOF

fi

sleep 3
#------------------------------------------------


# ------------------------------------------------
# avvio camera - reale: lancia i driver per la camera reale o il nodo per la fake)
if [[ "$use_real_camera" == "true" ]]; then
    echo "Avvio Intel Realsense..."

    gnome-terminal --tab --title="Intel Realsense Camera" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch realsense2_camera rs_launch.py; exec bash"
    sleep 5

    #nodo per vision..
else
    
    #fake camera: lancia il nodo che pubblica la posizione da file yaml
    echo "Avvio Fake Camera..."

    gnome-terminal --tab --title="Fake Camera" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch fake_camera fake_camera.launch.py yaml_path:=ws_ur5e_ballpool/src/camera_perception/fake_camera/config/fake_camera_config.yaml; exec bash"
    sleep 2
fi
# ------------------------------------------------


# ------------------------------------------------
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
    echo "Accendi il robot..."
    sleep 2

fi
# ------------------------------------------------



# ------------------------------------------------
# Avvio Driver
echo -e "\n================================================================="
echo -e "ATTENZIONE: ATTESA DI SINCRONIZZAZIONE CON L'UTENTE"
echo -e "================================================================="
echo -e "Verifica che il robot (simulato o reale) sia pronto per l'operatività.\nDevi averlo acceso cliccando con il mouse sul pulsante "Power Off e diventare verde Normal"\n"
echo -e "Premi un tasto per avviare i driver..."

# Mettiamo in pausa in attesa del segnale
read -n 1 -s -r

echo -e "\nProcedo con l'avvio del driver UR5e..."

if [[ "$scelta_URSim" =~ ^[sS][iI]?$ ]]; then
    gnome-terminal --tab --title="Driver UR5e" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch ur_robot_driver ur_control.launch.py   ur_type:=ur5e   robot_ip:=${robot_ip}  reverse_ip:=${reverse_ip}   launch_rviz:=false  description_file:=$(ros2 pkg prefix arm_description)/share/arm_description/urdf/arm_driver_wrapper.urdf.xacro; exec bash"
else
    gnome-terminal --tab --title="Driver UR5e" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch ur_robot_driver ur_control.launch.py   ur_type:=ur5e   robot_ip:=${robot_ip}  reverse_ip:=${reverse_ip}   launch_rviz:=false  description_file:=$(ros2 pkg prefix arm_description)/share/arm_description/urdf/arm_driver_wrapper.urdf.xacro kinematics_params_file:=$(ros2 pkg prefix arm_description)/share/arm_description/config/uclv_right_ur5e_kinematics.yaml; exec bash"
fi
# ------------------------------------------------




# ------------------------------------------------
# Avvio MoveIt
echo -e "\n================================================================="
echo -e "ATTENZIONE: ATTESA DI SINCRONIZZAZIONE CON L'UTENTE"
echo -e "================================================================="
echo -e "Verifica che il driver sia partito correttamente.\nDevi aver selezionato "External control" dal teach pendant e aver premuto Play.\nDal terminale del driver dovresti vedere la scritta 'Robot ready to receive commands'.\n"
echo -e "Premi un tasto per avviare Moveit..."

# Mettiamo in pausa in attesa del segnale
read -n 1 -s -r


echo "Avvio MoveIt con RViz..."
echo -e "ATTENZIONE:"
echo -e "-> Assicurati di aver decommentato il plugin ur_robot_driver nel file ur5e.ros2_control.xacro sezione hardware..."
sleep 2

gnome-terminal --tab --title="MoveGroup" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch moveit_config move_group.launch.py use_sim_time:=false; exec bash"
gnome-terminal --tab --title="Rviz" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch moveit_config moveit_rviz.launch.py use_sim_time:=false; exec bash"
sleep 10
# ------------------------------------------------


# ------------------------------------------------
# builder della scena
echo "Avvio Scene Builder..."
gnome-terminal --tab --title="Scene Builder" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run scene_description scene_builder; exec bash"

sleep 2
# ------------------------------------------------



if [[ "$execute_shot" == "true" ]]; then

    # ------------------------------------------------
    # gestione del logging
    if [["$logging_enabled" == "true" ]]; then

        #avvio il nodo di debug cartesiano che pubblica la posa del TCP del robot
        echo "Avvio Nodo di debug cartesiano..."
        gnome-terminal --tab --title="CartesianPublisher" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch logging_nodes cartesian_pub_launcher.launch.py; exec bash"
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
    # ------------------------------------------------


    # ------------------------------------------------
    # nodo che effettua il tiro vero e proprio
    if [["$logging_enable" == "true" ]]; then      
            # Con logging
            echo "Avvio Shot Planning con setup Logging..."      
            gnome-terminal --tab --title="Shot Planning" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning task_node --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/execution_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/shot_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/planning_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/logging_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/moveit_fix.yaml; exec bash"
        else
            # Senza  logging
            echo "Avvio Shot Planning senza logging..."
            gnome-terminal --tab --title="Shot Planning" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning task_node --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/execution_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/shot_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/planning_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/moveit_fix.yaml; exec bash"
        fi
    # ------------------------------------------------


    # ------------------------------------------------
    # game engine: se la variabile è vera, avvio il game engine reale, altrimenti quello fake
    if [[ "$use_real_game_engine" == "true" ]]; then
        
        echo "Avvio Game Engine Reale..."
        gnome-terminal --tab --title="Game Engine Reale" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning game_engine --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/game_engine_params.yaml; exec bash"
    else
        echo "Avvio Fake Game Engine..."
        gnome-terminal --tab --title="Fake Game Engine" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning fake_game_engine --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/game_engine_params.yaml; exec bash"
        
    fi
    # ------------------------------------------------
fi


echo "Tutti i nodi sono stati avviati!"