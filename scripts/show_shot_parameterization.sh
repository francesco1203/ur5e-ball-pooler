#!/bin/bash
# ANALISI PARAMETRICA DEL TIRO TRAMITE I FILE DI LOG 



# SCELTA DELLE ANALISI DA ESEGUIRE

# analisi bag-files
joints_analysis="true"             #true se vuoi aprire anche la finestra di analisi giunti
cartesian_analysis="true"          #true se vuoi aprire anche la finestra di analisi cartesiana
controller_analysis="true"         #true se vuoi aprire anche la finestra di plot dei dati del controller
torque_analysis="true"             #true se vuoi aprire anche la finestra di plot dei dati di coppia

# analisi ruckig csv
ruckig_plot="true"                   #true se vuoi aprire anche la finestra di plot dei dati Ruckig
cartesian_vs_ruckig_plot="true"      #true se vuoi aprire anche la finestra di plot dei dati cartesiani vs Ruckig



# RECUPERO DEL PERCORSO DEI BAG FILES, DEL CSV DI RUCKIG E DELLA CARTELLA DEGLI SCRIPT PYTHON

# Bagdata
BAGDATA_BASE_PATH="data/bagdata"
TEST_FOLDER_NAME="only_essential_logging"     #lo stesso che metti in "start_simulated_robot.sh" o "start_real_robot.sh" 

# Percorso completo della cartella del test
# Controllo esistenza cartella del test
TEST_FOLDER_PATH="$BAGDATA_BASE_PATH/$TEST_FOLDER_NAME"
if [ ! -d "$TEST_FOLDER_PATH" ]; then
    echo " Errore: La cartella del test non esiste:"
    echo "   $TEST_FOLDER_PATH"
    exit 1
fi

# Ricerca dell'ultimo bagfile registrato
# Usiamo ls -td per elencare le cartelle ordinate per data di modifica (la più recente per prima)
# Filtriamo per quelle che iniziano con "shot_" e prendiamo solo la prima
LATEST_SHOT_BAG_FOLDER=$(ls -td "$TEST_FOLDER_PATH"/shot_* 2>/dev/null | head -n 1)

# Se non trova nessuna cartella
if [ -z "$LATEST_SHOT_BAG_FOLDER" ]; then
    echo " Errore: Nessun bagfile trovato che inizi per 'shot_' in:"
    echo "   $TEST_FOLDER_PATH"
    exit 1
fi

echo "🔹 Trovato ultimo bag di tiro registrato: $LATEST_SHOT_BAG_FOLDER" #questo è il parametro di bag file da passare agli script di analisi python


#inserire il percorso corretto del CSV generato da Ruckig per la traiettoria del tiro
RUCKIG_CSV_PATH="data/csv/ruckig_logging/ruckig_trajectory_log.csv"

if [ ! -f "$RUCKIG_CSV_PATH" ]; then
    echo "Errore: File CSV non trovato in $RUCKIG_CSV_PATH"
    exit 1
fi

echo "🔹 Trovato log ruckig in: $RUCKIG_CSV_PATH" 


#path script python di analisi
ANALYSIS_SCRIPTS_PATH="data/analysis_scripts"


# LANCIO DEGLI SCRIPT SE RICHIESTO DALL'UTENTE

#  Joints Analyzer
if [[ "$joints_analysis" == "true" ]]; then

    #inserire il percorso corretto dello script Python per l'analisi dei giunti
    JOINTS_ANALYSIS_SCRIPT_PATH="${ANALYSIS_SCRIPTS_PATH}/plot_joints.py"

    if [ ! -f "$JOINTS_ANALYSIS_SCRIPT_PATH" ]; then
        echo "Errore: File Python non trovato in $JOINTS_ANALYSIS_SCRIPT_PATH"
        exit 1
    fi

    gnome-terminal -- bash -c "python3 "$JOINTS_ANALYSIS_SCRIPT_PATH" "$LATEST_SHOT_BAG_FOLDER"; exec bash"
fi


# Cartesian Analyzer
if [[ "$cartesian_analysis" == "true" ]]; then

    #inserire il percorso corretto dello script Python per l'analisi cartesiana
    CARTESIAN_ANALYSIS_SCRIPT_PATH="${ANALYSIS_SCRIPTS_PATH}/plot_cartesian.py"

    if [ ! -f "$CARTESIAN_ANALYSIS_SCRIPT_PATH" ]; then
        echo "Errore: File Python non trovato in $CARTESIAN_ANALYSIS_SCRIPT_PATH"
        exit 1
    fi

    #esecuzione
    gnome-terminal -- bash -c "python3 "$CARTESIAN_ANALYSIS_SCRIPT_PATH" "$LATEST_SHOT_BAG_FOLDER"; exec bash"
fi


# Controller Analyzer
if [[ "$controller_analysis" == "true" ]]; then

    #inserire il percorso corretto dello script Python per l'analisi del controller
    CONTROLLER_ANALYSIS_SCRIPT_PATH="${ANALYSIS_SCRIPTS_PATH}/plot_controller_state.py"

    if [ ! -f "$CONTROLLER_ANALYSIS_SCRIPT_PATH" ]; then
        echo "Errore: File Python non trovato in $CONTROLLER_ANALYSIS_SCRIPT_PATH"
        exit 1
    fi

    #esecuzione
    gnome-terminal -- bash -c "python3 "$CONTROLLER_ANALYSIS_SCRIPT_PATH" "$LATEST_SHOT_BAG_FOLDER"; exec bash"
fi


# Torque Analyzer
if [[ "$torque_analysis" == "true" ]]; then

    #inserire il percorso corretto dello script Python per l'analisi delle coppie
    TORQUE_ANALYSIS_SCRIPT_PATH="${ANALYSIS_SCRIPTS_PATH}/plot_torque.py"

    if [ ! -f "$TORQUE_ANALYSIS_SCRIPT_PATH" ]; then
        echo "Errore: File Python non trovato in $TORQUE_ANALYSIS_SCRIPT_PATH"
        exit 1
    fi

    #esecuzione
    gnome-terminal -- bash -c "python3 "$TORQUE_ANALYSIS_SCRIPT_PATH" "$LATEST_SHOT_BAG_FOLDER"; exec bash"
fi


# Plot Ruckig
if [[ "$ruckig_plot" == "true" ]]; then

    #inserire il percorso corretto dello script Python per l'analisi della traiettoria Ruckig
    RUCKIG_ANALYSIS_SCRIPT_PATH="${ANALYSIS_SCRIPTS_PATH}/plot_ruckig.py"

    if [ ! -f "$RUCKIG_ANALYSIS_SCRIPT_PATH" ]; then
        echo "Errore: File Python non trovato in $RUCKIG_ANALYSIS_SCRIPT_PATH"
        exit 1
    fi

    #esecuzione
    gnome-terminal -- bash -c "python3 "$RUCKIG_ANALYSIS_SCRIPT_PATH" "$RUCKIG_CSV_PATH"; exec bash"
    
fi


# cartesian vs Ruckig
if [[ "$cartesian_vs_ruckig_plot" == "true" ]]; then

    #inserire il percorso corretto dello script Python per l'analisi della traiettoria Ruckig
    CARTESIAN_VS_RUCKIG_ANALYSIS_SCRIPT_PATH="${ANALYSIS_SCRIPTS_PATH}/cartesian_vs_ruckig.py"

    if [ ! -f "$CARTESIAN_VS_RUCKIG_ANALYSIS_SCRIPT_PATH" ]; then
        echo "Errore: File Python non trovato in $CARTESIAN_VS_RUCKIG_ANALYSIS_SCRIPT_PATH"
        exit 1
    fi

    #esecuzione
    gnome-terminal -- bash -c "python3 "$CARTESIAN_VS_RUCKIG_ANALYSIS_SCRIPT_PATH" "$RUCKIG_CSV_PATH" "$LATEST_SHOT_BAG_FOLDER"; exec bash"
fi  
