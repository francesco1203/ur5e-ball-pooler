#!/bin/bash

# Finestra 1: Cartesian Analyzer
gnome-terminal -- bash -c "python3 src/execution_monitoring/data/cartesian_analyzer.py; exec bash"

# Finestra 2: Plot Ruckig
gnome-terminal -- bash -c "python3 src/shot_planning/debug/plot_ruckig.py; exec bash"