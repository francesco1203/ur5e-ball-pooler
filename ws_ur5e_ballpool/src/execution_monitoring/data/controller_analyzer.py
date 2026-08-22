#!/usr/bin/env python3
"""
plot_controller_position.py

Visualizza gli errori di posizione (e opzionalmente desired vs actual) per ogni giunto
a partire dal CSV generato da controller_state_logger.

"""

import sys

import pandas as pd
import matplotlib.pyplot as plt


def get_joint_names(columns):
    """Ricava i nomi dei giunti dalle colonne del CSV cercando il suffisso '_desired_pos'."""
    joints = []
    for col in columns:
        if col.endswith("_desired_pos"):
            joint_name = col[: -len("_desired_pos")]

            #rimuove left iniziale e joint finale
            if joint_name.startswith("left_"):
                joint_name = joint_name[len("left_"):]
            if joint_name.endswith("_joint"):
                joint_name = joint_name[: -len("_joint")]

            joints.append(joint_name)
    return joints



def plot_position(df, joints):
    """Una riga per giunto: errore di posizione (sinistra) e desired vs actual (destra)."""
    n = len(joints)
    fig, axes = plt.subplots(n, 2, figsize=(14, 3 * n), sharex=True)
 
    if n == 1:
        axes = axes.reshape(1, 2)
 
    for i, joint in enumerate(joints):
        ax_err = axes[i][0]
        ax_cmp = axes[i][1]
 
        ax_err.plot(df["time_sec"], df[f"left_{joint}_joint_error_pos"], color="crimson", linewidth=1)
        ax_err.axhline(0.0, color="black", linewidth=0.8, linestyle="--")
        ax_err.set_ylabel(f"{joint}\n")
        ax_err.grid(True, alpha=0.3)
 
        ax_cmp.plot(df["time_sec"], df[f"left_{joint}_joint_desired_pos"], label="Desired", color="green", linewidth=1.5)
        ax_cmp.plot(df["time_sec"], df[f"left_{joint}_joint_actual_pos"], label="Actual", color="black", linewidth=1, alpha=0.7)
        ax_cmp.grid(True, alpha=0.3)
 
        if i == 0:
            ax_err.set_title("Errore di posizione [rad]")
            ax_cmp.set_title("Desired position vs Actual position [rad]")
            ax_cmp.legend(loc="best")
 
    axes[-1][0].set_xlabel("Tempo [s]")
    axes[-1][1].set_xlabel("Tempo [s]")
 
    #fig.suptitle("Posizione per giunto: errore e confronto desired/actual", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
 


def main():
    
    # Se passi il file da riga di comando usa quello, altrimenti usa come default quello del tiro
    file_path = sys.argv[1] if len(sys.argv) > 1 else 'src/execution_monitoring/data/controller_logging/controller_log_4_shot.csv'

    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"Errore nella lettura del file {file_path}: {e}")
        return

    if "time_sec" not in df.columns:
        print("Errore: il CSV non contiene la colonna 'time_sec'. Controlla il file di input.")
        sys.exit(1)

    joints = get_joint_names(df.columns)
    if not joints:
        print("Errore: nessuna colonna '_desired_pos' trovata. Controlla l'intestazione del CSV.")
        sys.exit(1)

    print(f"Trovati {len(joints)} giunti: {joints}")
    print(f"Numero di campioni: {len(df)}")
    print(f"Durata registrata: {df['time_sec'].iloc[-1]:.3f} s")

    plot_position(df, joints)

    plt.show()


if __name__ == "__main__":
    main()