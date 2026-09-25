#!/usr/bin/env python3
"""
plot_torque.py

Visualizza i grafici dell'effort (coppia) dei giunti a partire dal bagfile ROS 2
per il topic degli attuatori (/joint_states).
Gestisce automaticamente l'assenza di dati di sforzo (es. in simulazione/mock).
"""

import sys
from pathlib import Path
import pandas as pd
import numpy as np
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
                
                # SE L'EFFORT E' PRESENTE E DELLA LUNGHEZZA CORRETTA, LO USIAMO
                if hasattr(msg, 'effort') and len(msg.effort) == len(joint_names):
                    efforts = list(msg.effort)
                else:
                    # Se l'array è vuoto (tipico in simulazione mock), usiamo NaN invece di 0.0
                    # così Matplotlib non disegna finte righe sullo zero.
                    efforts = [np.nan] * len(joint_names)
                
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
        
    return df, joint_names


def main():
    if len(sys.argv) < 2:
        print("Uso: python3 plot_torque.py <percorso_cartella_bag>")
        sys.exit(1)
        
    bag_path = sys.argv[1]
    topic_target = '/joint_states'

    print(f"Estrazione dati effort da: {bag_path} sul topic: {topic_target}...")
    df, joint_columns = extract_torque_from_bag(bag_path, topic_target)

    if df.empty:
        print("Nessun dato trovato o DataFrame vuoto. Uscita.")
        return

    # --- CONTROLLO DATI MANCANTI O NULLI (SIMULAZIONE) ---
    is_all_invalid = True
    for col in joint_columns:
        # Se c'è almeno un valore che NON è NaN e NON è 0.0, allora abbiamo dati reali
        if not (df[col].isna() | (df[col] == 0.0)).all():
            is_all_invalid = False
            break
            
    if is_all_invalid:
        print("\n" + "="*70)
        print(" ⚠️  ATTENZIONE: I dati di effort contengono solo ZERI o NaN.")
        print("     È probabile che questo bag provenga da una simulazione")
        print("     (es. mock_components) in cui la coppia non viene calcolata.")
        print("     I grafici verranno aperti, ma appariranno vuoti.")
        print("="*70 + "\n")

    # --- PLOTTING ---
    n_joints = len(joint_columns)
    fig, axes = plt.subplots(n_joints, 1, figsize=(10, 2.5 * n_joints), sharex=True)
    fig.canvas.manager.set_window_title(f'Analisi Coppie Giunti - {bag_path}')

    # Se c'è un solo giunto, normalizziamo axes in lista
    if n_joints == 1:
        axes = [axes]

    for ax, joint_name in zip(axes, joint_columns):
        # Utilizziamo 'dropna()' per non far arrabbiare matplotlib se ci sono NaN
        # Se tutti i dati sono NaN, matplotlib lascerà il riquadro pulito
        ax.plot(df["time_sec"], df[joint_name], linewidth=1.5, color='#1f77b4')
        ax.set_ylabel("Effort [Nm]")
        ax.set_title(joint_name)
        ax.grid(True, alpha=0.4)

    axes[-1].set_xlabel("Tempo [s]")
    fig.tight_layout()

    plt.show()


if __name__ == '__main__':
    main()