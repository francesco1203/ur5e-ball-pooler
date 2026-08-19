#!/usr/bin/env python3
"""
Visualizza i grafici dell'effort (coppia) dei giunti a partire dal CSV
prodotto dal nodo torque_logger.cpp.

Uso:
    python3 plot_torque.py percorso/al/file.csv
    python3 plot_torque.py percorso/al/file.csv --save grafico.png
"""

import argparse
import sys

import pandas as pd
import matplotlib.pyplot as plt


def main():

    # 1. ESTRAZIONE FILE DA DATI

    # Se passi il file da riga di comando usa quello, altrimenti usa come default quello del tiro
    file_path = sys.argv[1] if len(sys.argv) > 1 else 'src/execution_monitoring/data/torque_logging/torque_log_4_shot.csv'

    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"Errore nella lettura del file {file_path}: {e}")
        return

        
    if "time_sec" not in df.columns:
        print("Errore: il CSV non contiene la colonna 'time_sec'", file=sys.stderr)
        sys.exit(1)

    joint_columns = [col for col in df.columns if col != "time_sec"]
    if not joint_columns:
        print("Errore: nessuna colonna di giunto trovata nel CSV", file=sys.stderr)
        sys.exit(1)

    n_joints = len(joint_columns)
    fig, axes = plt.subplots(n_joints, 1, figsize=(10, 2.5 * n_joints), sharex=True)

    # Se c'è un solo giunto, axes non è una lista: normalizziamo
    if n_joints == 1:
        axes = [axes]

    for ax, joint_name in zip(axes, joint_columns):
        ax.plot(df["time_sec"], df[joint_name], linewidth=1)
        ax.set_ylabel("Effort [Nm]")
        ax.set_title(joint_name)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Tempo [s]")
    # fig.suptitle("Effort dei giunti nel tempo")
    fig.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()