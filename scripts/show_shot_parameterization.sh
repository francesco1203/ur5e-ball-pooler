#!/bin/bash
# ANALISI PARAMETRICA DEL TIRO TRAMITE I FILE DI LOG 


# MODO DI UTILIZZO:
# show_shot_parameterization.sh <COLORE>
#dove <COLORE> può essere:
#   - BLUE_SOLID
#   - RED_SOLID
#   - YELLOW_SOLID


# 1. ACQUISIZIONE DEL COLORE IN INGRESSO
if [ -z "$1" ]; then
    echo "Errore: Devi specificare il colore della palla bersaglio."
    echo "Uso: $0 <COLORE>"
    echo "Esempio: $0 BLUE_SOLID"
    exit 1
fi

TARGET_COLOR="$1"

# SCELTA DELLE ANALISI DA ESEGUIRE
joints_analysis="true"             
cartesian_analysis="true"          
controller_analysis="true"         
torque_analysis="true"             
ruckig_plot="true"                   
cartesian_vs_ruckig_plot="true"      


# RECUPERO DEL PERCORSO DEI BAG FILES, DEL CSV DI RUCKIG E DELLA CARTELLA DEGLI SCRIPT PYTHON

# Bagdata
BAGDATA_BASE_PATH="data/bagdata"
TEST_FOLDER_NAME="only_essential_logging"

TEST_FOLDER_PATH="$BAGDATA_BASE_PATH/$TEST_FOLDER_NAME"
if [ ! -d "$TEST_FOLDER_PATH" ]; then
    echo " Errore: La cartella del test non esiste: $TEST_FOLDER_PATH"
    exit 1
fi

# Ricerca dell'ultimo bagfile registrato associato a questo colore
LATEST_SHOT_BAG_FOLDER=$(ls -td "$TEST_FOLDER_PATH"/shot_${TARGET_COLOR}* 2>/dev/null | head -n 1)

if [ -z "$LATEST_SHOT_BAG_FOLDER" ]; then
    echo " Errore: Nessun bagfile trovato che inizi per 'shot_${TARGET_COLOR}' in:"
    echo "   $TEST_FOLDER_PATH"
    exit 1
fi

echo "🔹 Trovato ultimo bag di tiro registrato: $LATEST_SHOT_BAG_FOLDER" 


# Ricerca dell'ultimo file CSV di Ruckig associato a questo colore
RUCKIG_CSV_DIR="data/csv/ruckig_logging"
RUCKIG_CSV_PATH=$(ls -t "$RUCKIG_CSV_DIR"/ruckig_trajectory_log_${TARGET_COLOR}__*.csv 2>/dev/null | head -n 1)

if [ -z "$RUCKIG_CSV_PATH" ]; then
    echo "Errore: Nessun file CSV trovato per il colore ${TARGET_COLOR} in $RUCKIG_CSV_DIR"
    exit 1
fi

echo "🔹 Trovato log ruckig in: $RUCKIG_CSV_PATH" 


# Path script python di analisi
ANALYSIS_SCRIPTS_PATH="data/analysis_scripts"


# LANCIO DEGLI SCRIPT SE RICHIESTO DALL'UTENTE

# Joints Analyzer
if [[ "$joints_analysis" == "true" ]]; then
    JOINTS_ANALYSIS_SCRIPT_PATH="${ANALYSIS_SCRIPTS_PATH}/plot_joints.py"
    if [ ! -f "$JOINTS_ANALYSIS_SCRIPT_PATH" ]; then
        echo "Errore: File Python non trovato in $JOINTS_ANALYSIS_SCRIPT_PATH"
        exit 1
    fi
    gnome-terminal -- bash -c "python3 \"$JOINTS_ANALYSIS_SCRIPT_PATH\" \"$LATEST_SHOT_BAG_FOLDER\"; exec bash"
fi

# Cartesian Analyzer
if [[ "$cartesian_analysis" == "true" ]]; then
    CARTESIAN_ANALYSIS_SCRIPT_PATH="${ANALYSIS_SCRIPTS_PATH}/plot_cartesian.py"
    if [ ! -f "$CARTESIAN_ANALYSIS_SCRIPT_PATH" ]; then
        echo "Errore: File Python non trovato in $CARTESIAN_ANALYSIS_SCRIPT_PATH"
        exit 1
    fi
    gnome-terminal -- bash -c "python3 \"$CARTESIAN_ANALYSIS_SCRIPT_PATH\" \"$LATEST_SHOT_BAG_FOLDER\"; exec bash"
fi

# Controller Analyzer
if [[ "$controller_analysis" == "true" ]]; then
    CONTROLLER_ANALYSIS_SCRIPT_PATH="${ANALYSIS_SCRIPTS_PATH}/plot_controller_state.py"
    if [ ! -f "$CONTROLLER_ANALYSIS_SCRIPT_PATH" ]; then
        echo "Errore: File Python non trovato in $CONTROLLER_ANALYSIS_SCRIPT_PATH"
        exit 1
    fi
    gnome-terminal -- bash -c "python3 \"$CONTROLLER_ANALYSIS_SCRIPT_PATH\" \"$LATEST_SHOT_BAG_FOLDER\"; exec bash"
fi

# Torque Analyzer
if [[ "$torque_analysis" == "true" ]]; then
    TORQUE_ANALYSIS_SCRIPT_PATH="${ANALYSIS_SCRIPTS_PATH}/plot_torque.py"
    if [ ! -f "$TORQUE_ANALYSIS_SCRIPT_PATH" ]; then
        echo "Errore: File Python non trovato in $TORQUE_ANALYSIS_SCRIPT_PATH"
        exit 1
    fi
    gnome-terminal -- bash -c "python3 \"$TORQUE_ANALYSIS_SCRIPT_PATH\" \"$LATEST_SHOT_BAG_FOLDER\"; exec bash"
fi

# Plot Ruckig
if [[ "$ruckig_plot" == "true" ]]; then
    RUCKIG_ANALYSIS_SCRIPT_PATH="${ANALYSIS_SCRIPTS_PATH}/plot_ruckig.py"
    if [ ! -f "$RUCKIG_ANALYSIS_SCRIPT_PATH" ]; then
        echo "Errore: File Python non trovato in $RUCKIG_ANALYSIS_SCRIPT_PATH"
        exit 1
    fi
    gnome-terminal -- bash -c "python3 \"$RUCKIG_ANALYSIS_SCRIPT_PATH\" \"$RUCKIG_CSV_PATH\"; exec bash"
fi

# Cartesian vs Ruckig
if [[ "$cartesian_vs_ruckig_plot" == "true" ]]; then
    CARTESIAN_VS_RUCKIG_ANALYSIS_SCRIPT_PATH="${ANALYSIS_SCRIPTS_PATH}/cartesian_vs_ruckig.py"
    if [ ! -f "$CARTESIAN_VS_RUCKIG_ANALYSIS_SCRIPT_PATH" ]; then
        echo "Errore: File Python non trovato in $CARTESIAN_VS_RUCKIG_ANALYSIS_SCRIPT_PATH"
        exit 1
    fi
    gnome-terminal -- bash -c "python3 \"$CARTESIAN_VS_RUCKIG_ANALYSIS_SCRIPT_PATH\" \"$RUCKIG_CSV_PATH\" \"$LATEST_SHOT_BAG_FOLDER\"; exec bash"
fi