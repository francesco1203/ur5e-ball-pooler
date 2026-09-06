import sys
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from rosbags.highlevel import AnyReader

def extract_joints_from_bag(bag_path, topic_name):
    """Estrae i dati dei giunti e i timestamp da un bag ROS 2."""
    times = []
    joint_data = {}
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
                
                if not joint_names:
                    joint_names = msg.name
                    for name in joint_names:
                        joint_data[name] = []
                        
                times.append(t_sec)
                
                for i, name in enumerate(joint_names):
                    joint_data[name].append(msg.position[i])
                    
    except Exception as e:
        print(f"Errore nella lettura del bagfile {bag_path}: {e}")
        sys.exit(1)

    df = pd.DataFrame(joint_data)
    df['time_sec'] = times
    
    return df, joint_names

def main():
    bag_path = sys.argv[1]
    topic_target = '/joint_states'

    print(f"Estrazione dati da: {bag_path} sul topic: {topic_target}...")
    df, joint_cols = extract_joints_from_bag(bag_path, topic_target)

    if df.empty or not joint_cols:
        print("Nessun dato giunto trovato. Uscita.")
        return

    # --- 1. PULIZIA DEI TIMESTAMP ---
    df['dt'] = df['time_sec'].diff()
    df = df[(df['dt'].isna()) | (df['dt'] > 1e-3)].copy()

    # --- 2. RIMOZIONE TRANSITORIO INIZIALE (CROP) ---
    # Scartiamo i primissimi 5 campioni (circa 10-50ms) per eliminare 
    # assestamenti del controller e spike di avvio della registrazione.
    if len(df) > 10:
        df = df.iloc[5:].copy()

    # Ricalcoliamo il tempo partendo da 0 rispetto al nuovo primo campione
    df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]

    t = df['time_sec'].values
    
    # --- 3. CONFIGURAZIONE PLOT E FILTRI ---
    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.canvas.manager.set_window_title(f'Analisi Spazio Giunti - {bag_path}')

    filtering_position = False
    filtering_velocity = True
    filtering_acceleration = True
    filter_sg_window_length = 11  # dispari per il filtro di Savitzky-Golay
    filter_sg_polyorder = 3

    for joint in joint_cols:
        raw_pos = df[joint].values

        if filtering_position:
            # mode='nearest' previene gli sbandamenti polinomiali ai bordi
            plot_pos = savgol_filter(raw_pos, window_length=filter_sg_window_length, polyorder=filter_sg_polyorder, mode='nearest')
        else:
            plot_pos = raw_pos

        # Derivata 1: Velocità
        vel = np.gradient(raw_pos, t)          

        if filtering_velocity:
            vel = savgol_filter(vel, window_length=filter_sg_window_length, polyorder=filter_sg_polyorder, mode='nearest')

        # Derivata 2: Accelerazione
        acc = np.gradient(vel, t)          

        if filtering_acceleration:
            acc = savgol_filter(acc, window_length=filter_sg_window_length, polyorder=filter_sg_polyorder, mode='nearest')

        axs[0].plot(t, plot_pos, label=joint)
        axs[1].plot(t, vel, label=f'vel_{joint}')
        axs[2].plot(t, acc, label=f'acc_{joint}')

    axs[0].set_ylabel('Posizione [rad]')
    if filtering_position:
        axs[0].set_title('Posizione dei Giunti nel Tempo (filtrato SG)')
    else:
        axs[0].set_title('Posizione dei Giunti nel Tempo')
    axs[0].grid(True)
    axs[0].legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))

    axs[1].set_ylabel('Velocità [rad/s]')
    if filtering_velocity:
        axs[1].set_title('Velocità dei Giunti nel Tempo (filtrato SG)')
    else:
        axs[1].set_title('Velocità dei Giunti nel Tempo')
    axs[1].grid(True)
    axs[1].legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))

    axs[2].set_ylabel('Accelerazione [rad/s²]')
    axs[2].set_xlabel('Tempo [s]')
    if filtering_acceleration:
        axs[2].set_title('Accelerazione dei Giunti nel Tempo (filtrato SG)')
    else:
        axs[2].set_title('Accelerazione dei Giunti nel Tempo')
    axs[2].grid(True)
    axs[2].legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()