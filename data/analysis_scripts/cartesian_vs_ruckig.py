#!/usr/bin/env python3
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Librerie native ROS 2 per leggere i bag file
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def extract_pose_from_bag(bag_path, topic_name):
    """Estrae i dati di posizione cartesiana e i timestamp da un bag ROS 2 in modo nativo."""
    times, xs, ys, zs = [], [], [], []
    
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Errore nell'apertura del bag: {e}")
        sys.exit(1)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    if topic_name not in type_map:
        print(f"Errore: Il topic '{topic_name}' non è presente nel bagfile.")
        return pd.DataFrame()
        
    msg_type_str = type_map[topic_name]
    msg_type = get_message(msg_type_str)

    storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
    reader.set_filter(storage_filter)

    while reader.has_next():
        topic, rawdata, timestamp = reader.read_next()
        msg = deserialize_message(rawdata, msg_type)
        t_sec = timestamp / 1e9 
        
        xs.append(msg.pose.position.x)
        ys.append(msg.pose.position.y)
        zs.append(msg.pose.position.z)
        times.append(t_sec)

    df = pd.DataFrame({'time_sec': times, 'x': xs, 'y': ys, 'z': zs})
    
    if not df.empty:
        df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]
        
    return df

def main():
    # Se passi i percorsi da terminale: python script.py <file_csv_ideale> <cartella_bag_raw>
    if len(sys.argv) < 3:
        print("Uso: python3 compare_ruckig.py <file_ideale.csv> <cartella_bag_reale>")
        sys.exit(1)

    ideal_file_path = sys.argv[1]
    raw_bag_path = sys.argv[2]
    topic_target = '/tcp_pose_broadcaster/pose'

    # --- 1. LETTURA DATI IDEALI (RUCKIG da CSV) ---
    try:
        df_ideal = pd.read_csv(ideal_file_path)
    except Exception as e:
        print(f"Errore nella lettura del file ideale CSV: {e}")
        return

    # Presumendo che le colonne siano: Time_s, Position_m, Velocity_ms, Acceleration_ms2
    t_ideal = df_ideal.iloc[:, 0].values
    pos_ideal = df_ideal.iloc[:, 1].values
    vel_ideal = df_ideal.iloc[:, 2].values
    acc_ideal = df_ideal.iloc[:, 3].values

    # --- 2. LETTURA DATI ESECUZIONE (RAW da BAGFILE) ---
    print(f"Estrazione dati reali da: {raw_bag_path}...")
    df_raw = extract_pose_from_bag(raw_bag_path, topic_target)

    if df_raw.empty:
        print("Nessun dato estratto dal bag. Uscita.")
        return

    # Pulizia timestamp per evitare divisioni per zero o picchi irreali (burst initiali)
    df_raw['dt'] = df_raw['time_sec'].diff()
    df_raw = df_raw[(df_raw['dt'].isna()) | (df_raw['dt'] > 1e-3)].copy()
    df_raw['time_sec'] = df_raw['time_sec'] - df_raw['time_sec'].iloc[0]

    t_raw = df_raw['time_sec'].values
    x, y, z = df_raw['x'].values, df_raw['y'].values, df_raw['z'].values

    # Calcoli reali
    dist_raw = np.sqrt((x - x[0])**2 + (y - y[0])**2 + (z - z[0])**2)

    # Velocità pura (Delta Spazio / Delta Tempo)
    dt = np.diff(t_raw)
    vel_raw = np.diff(dist_raw) / dt
    t_vel_raw = t_raw[:-1]

    # Accelerazione pura
    dt_vel = np.diff(t_vel_raw)
    acc_raw = np.diff(vel_raw) / dt_vel
    t_acc_raw = t_vel_raw[:-1]

    # --- 3. PLOTTING SOVRAPPOSTO ---
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title('Confronto: Ideale vs Esecuzione Reale')

    # Subplot 1: Posizione
    axs[0].plot(t_ideal, pos_ideal, 'g-', label='Traiettoria Ideale (Ruckig)', linewidth=2)
    axs[0].plot(t_raw, dist_raw, 'ko', label='Campioni Reali (Bag Logger)', markersize=4, alpha=0.6)
    axs[0].set_ylabel('Distanza [m]')
    axs[0].set_title('Confronto Posizione')
    axs[0].grid(True); axs[0].legend()

    # Subplot 2: Velocità
    axs[1].plot(t_ideal, vel_ideal, color='orange', label='Velocità Ideale', linewidth=2)
    axs[1].plot(t_vel_raw, vel_raw, 'ko', label='Velocità Calcolata', markersize=4, alpha=0.6)
    axs[1].set_ylabel('Velocità [m/s]')
    axs[1].set_title('Confronto Velocità')
    axs[1].grid(True); axs[1].legend()

    # Subplot 3: Accelerazione
    axs[2].plot(t_ideal, acc_ideal, 'r-', label='Accelerazione Ideale', linewidth=2)
    axs[2].plot(t_acc_raw, acc_raw, 'ko', label='Accelerazione Calcolata', markersize=4, alpha=0.6)
    axs[2].set_ylabel('Accelerazione [m/s²]')
    axs[2].set_xlabel('Tempo [s]')
    axs[2].set_title('Confronto Accelerazione')
    axs[2].axhline(0, color='black', linewidth=0.8, linestyle=':')
    axs[2].grid(True); axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()