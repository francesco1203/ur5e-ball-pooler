#!/bin/bash

# ANALISI PARAMETRICA DEL TIRO TRAMITE I FILE DI LOG CSV


# Parametri di personalizzazione visualizzazione
joints_analysis="false"             #true se vuoi aprire anche la finestra di analisi giunti
cartesian_analysis="true"           #true se vuoi aprire anche la finestra di analisi cartesiana
ruckig_plot="true"                  #true se vuoi aprire anche la finestra di plot dei dati Ruckig
cartesian_vs_ruckig_plot="true"     #true se vuoi aprire anche la finestra di plot dei dati cartesiani vs Ruckig
controller_analysis="true"          #true se vuoi aprire anche la finestra di plot dei dati del controller
torque_analysis="false"             #true se vuoi aprire anche la finestra di plot dei dati di coppia

# Finestra 1: Joints Analyzer
if [[ "$joints_analysis" == "true" ]]; then
    gnome-terminal -- bash -c "python3 src/execution_monitoring/csv_subscribers_nodes/data/joints_analyzer.py; exec bash"
fi

# Finestra 2: Cartesian Analyzer
if [[ "$cartesian_analysis" == "true" ]]; then
    gnome-terminal -- bash -c "python3 src/execution_monitoring/csv_subscribers_nodes/data/cartesian_analyzer.py; exec bash"
fi

# Finestra 3: Plot Ruckig
if [[ "$ruckig_plot" == "true" ]]; then
    gnome-terminal -- bash -c "python3 src/shot_planning/debug/plot_ruckig.py; exec bash"
fi


# Finestra 4: cartesian vs Ruckig
if [[ "$cartesian_vs_ruckig_plot" == "true" ]]; then
    gnome-terminal -- bash -c "python3 src/shot_planning/debug/cartesian_vs_ruckig.py; exec bash"
fi  


# Finestra 5: Controller Analyzer
if [[ "$controller_analysis" == "true" ]]; then
    gnome-terminal -- bash -c "python3 src/execution_monitoring/csv_subscribers_nodes/data/controller_analyzer.py; exec bash"
fi

# Finestra 6: Torque Analyzer
if [[ "$torque_analysis" == "true" ]]; then
    gnome-terminal -- bash -c "python3 src/execution_monitoring/csv_subscribers_nodes/data/torque_analyzer.py; exec bash"
fi