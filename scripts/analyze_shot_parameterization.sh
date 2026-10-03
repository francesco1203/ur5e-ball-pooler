#!/bin/bash
# ANALISI COMPLETA DELLA SEQUENZA DI TIRO

if [ -z "$1" ]; then
    echo "Uso: $0 <COLORE> [PREFISSO]"
    echo "Esempio: $0 BLUE_SOLID test_completo"
    exit 1
fi

TARGET_COLOR="$1"
PREFIX="${2}"

BAGDATA_BASE_PATH="data/bagdata/only_essential_logging"
ANALYSIS_SCRIPTS_PATH="data/analysis_scripts"

# Flag principale
save_results="true"

# Creazione del nome della cartella radice per questa analisi
CURRENT_TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
if [ -z "$PREFIX" ]; then
    ROOT_RESULT_FOLDER="${TARGET_COLOR}_${CURRENT_TIMESTAMP}"
else
    ROOT_RESULT_FOLDER="${PREFIX}_${TARGET_COLOR}_${CURRENT_TIMESTAMP}"
fi

echo "================================================================"
echo " INIZIO ANALISI COMPLETA - TARGET: $TARGET_COLOR"
echo " I risultati verranno salvati in: data/results/$ROOT_RESULT_FOLDER/"
echo "================================================================"

# Funzione per trovare l'ultimo bag di una specifica fase
find_latest_bag() {
    local phase_prefix=$1
    ls -td "${BAGDATA_BASE_PATH}"/${phase_prefix}_${TARGET_COLOR}* 2>/dev/null | head -n 1
}

# Funzione per lanciare lo script (usa cartelle nidificate!)
launch() {
    local script_name=$1
    local bag_path=$2
    local phase_folder=$3
    local extra_arg=$4 # opzionale per ruckig

    if [ -n "$bag_path" ]; then
        if [ "$save_results" == "true" ]; then
            # Notare che passiamo "ROOT/PHASE" come sottocartella
            local save_path="${ROOT_RESULT_FOLDER}/${phase_folder}"
            
            if [ -n "$extra_arg" ]; then
                python3 "${ANALYSIS_SCRIPTS_PATH}/${script_name}" "$extra_arg" "$bag_path" --save "$save_path" &
            else
                python3 "${ANALYSIS_SCRIPTS_PATH}/${script_name}" "$bag_path" --save "$save_path" &
            fi
        else
            gnome-terminal -- bash -c "python3 \"${ANALYSIS_SCRIPTS_PATH}/${script_name}\" \"$bag_path\"; exec bash"
        fi
    fi
}

# =========================================================================
# FASE 1: PRE-APPROACH (Movimento Giunti)
# =========================================================================
BAG_PHASE1=$(find_latest_bag "preapproach")
if [ -n "$BAG_PHASE1" ]; then
    echo "▶ Analisi Fase 1: Pre-Approach..."
    launch "plot_joints.py" "$BAG_PHASE1" "01_pre_approach"
    launch "plot_controller_state.py" "$BAG_PHASE1" "01_pre_approach"
fi

# =========================================================================
# FASE 2: APPROACH (Movimento Cartesiano)
# =========================================================================
BAG_PHASE2=$(find_latest_bag "approach")
if [ -n "$BAG_PHASE2" ]; then
    echo "▶ Analisi Fase 2: Approach..."
    launch "plot_cartesian.py" "$BAG_PHASE2" "02_approach"
    launch "plot_joints.py" "$BAG_PHASE2" "02_approach"
fi

# =========================================================================
# FASE 3: BACK SHOT (Movimento Cartesiano)
# =========================================================================
BAG_PHASE3=$(find_latest_bag "back_shot")
if [ -n "$BAG_PHASE3" ]; then
    echo "▶ Analisi Fase 3: Back Shot..."
    launch "plot_cartesian.py" "$BAG_PHASE3" "03_back_shot"
    launch "plot_joints.py" "$BAG_PHASE3" "03_back_shot"
fi

# =========================================================================
# FASE 4: SHOT EXECUTION (Impatto e Ruckig)
# =========================================================================
BAG_PHASE4=$(find_latest_bag "shot")
if [ -n "$BAG_PHASE4" ]; then
    echo "▶ Analisi Fase 4: Esecuzione Tiro..."
    
    # Trova il CSV Ruckig per il confronto
    RUCKIG_CSV=$(ls -t data/csv/ruckig_logging/ruckig_trajectory_log_${TARGET_COLOR}__*.csv 2>/dev/null | head -n 1)
    
    launch "plot_cartesian.py" "$BAG_PHASE4" "04_shot_execution"
    launch "plot_joints.py" "$BAG_PHASE4" "04_shot_execution"
    launch "plot_wrench.py" "$BAG_PHASE4" "04_shot_execution"
    launch "plot_torque.py" "$BAG_PHASE4" "04_shot_execution"
    launch "plot_controller_state.py" "$BAG_PHASE4" "04_shot_execution"

    if [ -n "$RUCKIG_CSV" ]; then
        # Nota: per cartesian_vs_ruckig passiamo il CSV come extra arg
        launch "cartesian_vs_ruckig.py" "$BAG_PHASE4" "04_shot_execution" "$RUCKIG_CSV"
        # Per plot_ruckig passiamo solo il CSV (hack rapido invertendo l'argomento per la funzione launch)
        launch "plot_ruckig.py" "$RUCKIG_CSV" "04_shot_execution"
    fi
fi

# =========================================================================
# FASE 5: GET HIGH (Movimento Cartesiano)
# =========================================================================
BAG_PHASE5=$(find_latest_bag "get_high")
if [ -n "$BAG_PHASE5" ]; then
    echo "▶ Analisi Fase 5: Get High..."
    launch "plot_cartesian.py" "$BAG_PHASE5" "05_get_high"
fi

# =========================================================================
# FASE 6: AWAY FROM TABLE (Movimento Giunti)
# =========================================================================
BAG_PHASE6=$(find_latest_bag "away_from_table")
if [ -n "$BAG_PHASE6" ]; then
    echo "▶ Analisi Fase 6: Away From Table..."
    launch "plot_joints.py" "$BAG_PHASE6" "06_away_from_table"
fi


# Chiusura
if [ "$save_results" == "true" ]; then
    echo " "
    echo "⏳ Elaborazione dei grafici in background in corso..."
    wait
    echo "✅ Analisi completa generata con successo in data/results/$ROOT_RESULT_FOLDER/"
fi