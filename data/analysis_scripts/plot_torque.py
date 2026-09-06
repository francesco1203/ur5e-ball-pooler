#!/usr/bin/env python3
"""
plot_torque.py

Visualizza i grafici dell'effort (coppia) dei giunti a partire dal bagfile ROS 2
per il topic degli attuatori MuJoCo (/mujoco_actuators_states) o simili.
"""

import sys
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader


def extract_torque_from_bag(bag_path, topic_name):
    """Estrae i dati di effort/coppia e i timestamp da un bag ROS 2."""
    times = []
    torque_data = {}
    joint_names = []
    
    try:
        bagpath = Path(bag_path)
        with AnyReader([bagpath]) as reader:
            topic_connections = [conn for conn in reader.connections if conn.topic == topic_name]
            if not topic_connections:
                print(f"Errore: Il topic '{topic_name}' non è presente nel bagfile.")
                sys.exit(1)

            for connection, timestamp, rawdata in reader.messages(connections=topic_connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                t_sec = timestamp / 1e9 
                
                # Inizializza le colonne basandosi sui nomi dei giunti del primo messaggio
                if not joint_names:
                    if hasattr(msg, 'name'):
                        joint_names = msg.name
                    else:
                        # Fallback se il tipo di messaggio usa una struttura diversa
                        joint_names = [f"actuator_{i}" for i in range(len(msg.effort))]
                        
                    for name in joint_names:
                        torque_data[name] = []
                        
                times.append(t_sec)
                
                # Popola le liste con l'effort (gestendo eventuali campi vuoti)
                efforts = msg.effort if hasattr(msg, 'effort') and len(msg.effort) == len(joint_names) else [0.0] * len(joint_names)
                
                for i, name in enumerate(joint_names):
                    torque_data[name].append(efforts[i])
                    
    except Exception as e:
        print(f"Errore nella lettura del bagfile {bag_path}: {e}")
        sys.exit(1)

    # Creazione DataFrame
    df = pd.DataFrame(torque_data)
    df['time_sec'] = times
    
    if not df.empty:
        df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]
        
    return df


def main():
    # Se passi il file/cartella da riga di comando usa quello, altrimenti usa un default
    bag_path = sys.argv[1]
    topic_target = '/mujoco_actuators_states'

    print(f"Estrazione dati effort da: {bag_path} sul topic: {topic_target}...")
    df = extract_torque_from_bag(bag_path, topic_target)

    if df.empty:
        print("Nessun dato di effort trovato o DataFrame vuoto. Uscita.")
        return

    if "time_sec" not in df.columns:
        print("Errore: il DataFrame non contiene la colonna 'time_sec'", file=sys.stderr)
        sys.exit(1)

    joint_columns = [col for col in df.columns if col != "time_sec"]
    if not joint_columns:
        print("Errore: nessuna colonna di giunto trovata", file=sys.stderr)
        sys.exit(1)

    n_joints = len(joint_columns)
    fig, axes = plt.subplots(n_joints, 1, figsize=(10, 2.5 * n_joints), sharex=True)

    # Se c'è un solo giunto, normalizziamo axes in lista
    if n_joints == 1:
        axes = [axes]

    for ax, joint_name in zip(axes, joint_columns):
        ax.plot(df["time_sec"], df[joint_name], linewidth=1)
        ax.set_ylabel("Effort [Nm]")
        ax.set_title(joint_name)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Tempo [s]")
    fig.tight_layout()

    plt.show()


if __name__ == '__main__':
    main()