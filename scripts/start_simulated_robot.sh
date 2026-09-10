#!/bin/bash

# PARAMETRI DI PERSONALIZZAZIONE ESECUZIONE OFF-LINE

#simulazione
open_rviz_when_using_mujoco="false"            #true se vuoi aprire anche RViz quando usi MuJoCo, false se vuoi aprire solo MuJoCo
billiard_position="s"                          #s = sinistra, cs = centro-sinistra
build_scene_rviz="true"                        #true se vuoi costruire la scena in RViz, indicando gli ostacoli in moveit

#esecuzione tiro
execute_shot="true"                            #false se vuoi solo fare visualizzazione della scena e non eseguire il tiro (utile in fase di debug e setup)
use_real_game_engine="true"                    #true se vuoi usare il game engine reale, false se vuoi usare quello fake

#logging brutal vs only essential
logging_enable="false"                                         #true se vuoi fare logging
brutal_logging="false"                                        #true se vuoi fare logging di tutti i dati, false se vuoi fare logging solo dei dati essenziali
brutal_logging_folder="brutal_logging"                        #nome della cartella di logging, che verrà creata in data/bagdata/<logging_folder_title>
only_essential_logging_folder="only_essential_logging"        #nome della cartella di logging, che verrà creata in data/bagdata/<logging_folder_title>



# Verifica preliminare della cartella di lavoro
if [ ! -d "ws_ur5e_ballpool/install" ]; then
    echo "Errore: Cartella 'install' non trovata!"
    echo "Assicurati di aver buildato il workspace ROS 2 e di lanciare questo script dalla root del progetto."
    exit 1
fi


echo "========================================"
echo "      CONFIGURAZIONE AVVIO ROS 2        "
echo "========================================"
read -p "Vuoi usare MuJoCo? (s/n): " scelta_mujoco


# Gestione della scelta con if-else
if [[ "$scelta_mujoco" =~ ^[sS][iI]?$ ]]; then

    # devo generare la scena completa con le palline per MuJoCo con lo script autocreate_complete_scene.py
    case "$billiard_position" in
        "s")
            python3 ws_ur5e_ballpool/src/camera_perception/fake_camera/mujoco_automation/autocreate_complete_scene.py --yaml_path ws_ur5e_ballpool/src/camera_perception/fake_camera/config/fake_camera_config_sx.yaml
            ;;
        "cs")
            python3 ws_ur5e_ballpool/src/camera_perception/fake_camera/mujoco_automation/autocreate_complete_scene.py --yaml_path ws_ur5e_ballpool/src/camera_perception/fake_camera/config/fake_camera_config_center_sx.yaml
            ;;
        *)
            # Caso di default se la variabile non corrisponde a nessuno dei precedenti
            echo "Errore: posizione '$billiard_position' non riconosciuta."
            exit 1
            ;;
    esac
    

    # Controlliamo se lo script python è andato a buon fine ($? == 0)
    if [ $? -ne 0 ]; then
        echo "ERRORE: Generazione della scena MuJoCo fallita! Interruzione."
        exit 1
    fi
    echo -e "Scena MuJoCo generata con successo!\n"


    #avvio del simulatore vero e proprio con MuJoCo e MoveIt
    echo "Avvio MuJoCo con MoveIt..."
    echo -e "ATTENZIONE:"
    echo -e "-> Assicurati di aver decommentato il plugin MuJoCo nel file arm_ur5e.ros2_control.xacro sezione hardware...\n"
    
    sleep 2

    if [[ "$open_rviz_when_using_mujoco" == "true" ]]; then
        gnome-terminal --tab --title="Moveit+MuJoCo+Rviz" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch moveit_config mujoco_and_rviz_demo.launch.py; exec bash"
        sleep 10
    else
        gnome-terminal --tab --title="Moveit+MuJoCo" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch moveit_config mujoco_demo.launch.py; exec bash"
        sleep 5
    fi

else
    echo "Avvio MoveIt con RViz..."
    echo -e "ATTENZIONE:"
    echo -e "-> Assicurati di aver decommentato il plugin FakeHardware nel file arm_ur5e.ros2_control.xacro sezione hardware...\n"
    sleep 2

    gnome-terminal --tab --title="MoveIt+Rviz" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 launch moveit_config demo.launch.py; exec bash"
    sleep 10
fi


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
if [[ "$build_scene_rviz" == "true" ]]; then
    echo "Avvio Scene Builder..."
    gnome-terminal --tab --title="Scene Builder" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run scene_description scene_builder; exec bash"

    sleep 2
fi



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
    

    # nodo che effettua il tiro (distinguo i casi con MuJoCo e senza, con logging e senza logging)
    if [[ "$scelta_mujoco" =~ ^[sS][iI]?$ ]]; then

        if [["$logging_enabled" == "true" ]]; then      
            # MuJoCo + logging
            echo "Avvio Shot Planning con setup UseMuJoCo+Logging..."      
            gnome-terminal --tab --title="Shot Planning" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning task_node --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/execution_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/shot_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/planning_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/logging_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/moveit_fix.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/using_mujoco.yaml; exec bash"
        else
            # MuJoCo senza logging
            echo "Avvio Shot Planning con setup UseMuJoCo..."
            gnome-terminal --tab --title="Shot Planning" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning task_node --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/execution_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/shot_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/planning_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/moveit_fix.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/using_mujoco.yaml; exec bash"
        fi

    else

        if [["$logging_enabled" == "true" ]]; then  
            # senza MuJoCo + logging
            echo "Avvio Shot Planning con setup Logging..."
            gnome-terminal --tab --title="Shot Planning" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning task_node --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/execution_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/shot_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/planning_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/logging_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/moveit_fix.yaml; exec bash"
        else
            # senza MuJoCo senza logging
            echo "Avvio Shot Planning senza setup Logging..."
            gnome-terminal --tab --title="Shot Planning" -- bash -c "source ws_ur5e_ballpool/install/setup.bash && ros2 run shot_planning task_node --ros-args --params-file ws_ur5e_ballpool/src/shot_planning/config/execution_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/shot_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/planning_params.yaml --params-file ws_ur5e_ballpool/src/shot_planning/config/moveit_fix.yaml; exec bash"
        fi
        
    fi
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