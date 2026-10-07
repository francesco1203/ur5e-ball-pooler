#!/usr/bin/env python3
"""
cartesian_vs_ruckig.py

Visualizza i dati di posizione, velocità e accelerazione cartesiana in linea retta, 
estraendo i dati da un bagfile, confrontando con i dati ideali generati da Ruckig.
Supporta il salvataggio automatico in data/results usando il flag --save.
"""

import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from pathlib import Path

# Librerie native ROS 2 per leggere i bag file
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def extract_kinematics_from_bag(bag_path, pose_topic, twist_topic):
    """Estrae i dati di posizione e twist cartesiano da un bag ROS 2 in modo nativo e li allinea."""
    pose_times, xs, ys, zs = [], [], [], []
    twist_times, vxs, vys, vzs = [], [], [], []
    
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
    
    if pose_topic not in type_map or twist_topic not in type_map:
        print(f"Errore: Assicurati che i topic '{pose_topic}' e '{twist_topic}' siano nel bagfile.")
        return pd.DataFrame()
        
    msg_types = {
        pose_topic: get_message(type_map[pose_topic]),
        twist_topic: get_message(type_map[twist_topic])
    }

    storage_filter = rosbag2_py.StorageFilter(topics=[pose_topic, twist_topic])
    reader.set_filter(storage_filter)

    while reader.has_next():
        topic, rawdata, timestamp = reader.read_next()
        msg = deserialize_message(rawdata, msg_types[topic])
        t_sec = timestamp / 1e9 
        
        if topic == pose_topic:
            xs.append(msg.pose.position.x)
            ys.append(msg.pose.position.y)
            zs.append(msg.pose.position.z)
            pose_times.append(t_sec)
        elif topic == twist_topic:
            vxs.append(msg.twist.linear.x)
            vys.append(msg.twist.linear.y)
            vzs.append(msg.twist.linear.z)
            twist_times.append(t_sec)

    df_pose = pd.DataFrame({'time_sec': pose_times, 'x': xs, 'y': ys, 'z': zs}).sort_values('time_sec')
    df_twist = pd.DataFrame({'time_sec': twist_times, 'vx': vxs, 'vy': vys, 'vz': vzs}).sort_values('time_sec')
    
    if df_pose.empty or df_twist.empty:
        return pd.DataFrame()

    df = pd.merge_asof(df_pose, df_twist, on='time_sec', direction='nearest')
    
    if not df.empty:
        df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]
        
    return df

def main():
    print("Cartesian vs Ruckig: Confronto tra traiettoria ideale e reale")

    # --- LETTURA ARGOMENTI E FLAG --save ---
    args = sys.argv[1:]
    
    save_results = '--save' in args
    save_subdir = None
    
    if save_results:
        idx = args.index('--save')
        # Verifica se l'utente ha passato il nome della sottocartella
        if idx + 1 < len(args) and not args[idx+1].startswith('-'):
            save_subdir = args[idx + 1]
            args.pop(idx + 1)
        args.pop(idx)
        
    if len(args) < 2:
        print("Uso: python3 cartesian_vs_ruckig.py <file_ideale.csv> <cartella_bag_reale> [--save <sottocartella>]")
        sys.exit(1)

    ideal_file_path = args[0]
    raw_bag_path = args[1]
    
    topic_pose = '/tcp_pose_broadcaster/pose'
    topic_twist = '/tcp_twist'

    # --- 1. LETTURA DATI IDEALI (RUCKIG da CSV) ---
    try:
        df_ideal = pd.read_csv(ideal_file_path)
    except Exception as e:
        print(f"Errore nella lettura del file ideale CSV: {e}")
        return

    t_ideal = df_ideal.iloc[:, 0].values
    pos_ideal = df_ideal.iloc[:, 1].values
    vel_ideal = df_ideal.iloc[:, 2].values
    acc_ideal = df_ideal.iloc[:, 3].values

    # --- 2. LETTURA DATI ESECUZIONE (RAW da BAGFILE) ---
    print(f"Estrazione dati reali da: {raw_bag_path}...")
    df_raw = extract_kinematics_from_bag(raw_bag_path, topic_pose, topic_twist)

    if df_raw.empty:
        print("Nessun dato estratto dal bag. Uscita.")
        return

    # Controllo validità velocità
    is_all_invalid = True
    for col in ['vx', 'vy', 'vz']:
        if not (df_raw[col].isna() | (df_raw[col] == 0.0)).all():
            is_all_invalid = False
            break
            
    if is_all_invalid:
        print("\n" + "="*75)
        print(" ⚠️  ATTENZIONE: Le velocità reali contengono solo ZERI o NaN.")
        print("="*75 + "\n")
    
    # Pulizia timestamp
    df_raw['dt'] = df_raw['time_sec'].diff()
    df_raw = df_raw[(df_raw['dt'].isna()) | (df_raw['dt'] > 1e-3)].copy()
    df_raw['time_sec'] = df_raw['time_sec'] - df_raw['time_sec'].iloc[0]

    # Rimozione coda statica
    temp_x, temp_y, temp_z = df_raw['x'].values, df_raw['y'].values, df_raw['z'].values
    temp_dist = np.sqrt((temp_x - temp_x[0])**2 + (temp_y - temp_y[0])**2 + (temp_z - temp_z[0])**2)
    dist_diff = np.abs(np.diff(temp_dist, prepend=0.0))
    
    active_indices = np.where(dist_diff > 1e-4)[0]
    
    if len(active_indices) > 0:
        last_active_idx = active_indices[-1]
        buffer_samples = 10
        cut_idx = min(last_active_idx + buffer_samples, len(df_raw))
        df_raw = df_raw.iloc[:cut_idx].copy()
    else:
        print("Nessun movimento cartesiano rilevato nell'intero log reale.")

    df_raw['time_sec'] = df_raw['time_sec'] - df_raw['time_sec'].iloc[0]

    t_raw = df_raw['time_sec'].values
    x, y, z = df_raw['x'].values, df_raw['y'].values, df_raw['z'].values
    vx, vy, vz = df_raw['vx'].values, df_raw['vy'].values, df_raw['vz'].values

    dist_raw = np.sqrt((x - x[0])**2 + (y - y[0])**2 + (z - z[0])**2)
    vel_raw = np.sqrt(vx**2 + vy**2 + vz**2)

    # Filtraggio Savitzky-Golay
    filtering_distance = False
    filtering_velocity = False
    filtering_acceleration = True
    
    wl = 9
    po = 3
    dt_medio = np.mean(np.diff(t_raw))

    plot_dist = savgol_filter(dist_raw, window_length=wl, polyorder=po, mode='nearest') if filtering_distance else dist_raw
    label_dist = 'Campioni Reali (Filtrati SG)' if filtering_distance else 'Campioni Reali (Raw)'

    plot_vel = savgol_filter(vel_raw, window_length=wl, polyorder=po, mode='nearest') if filtering_velocity else vel_raw
    label_vel = 'Velocità Calcolata (Filtrata SG)' if filtering_velocity else 'Velocità Calcolata (Raw)'

    if filtering_acceleration:
        plot_acc = savgol_filter(plot_vel, window_length=wl, polyorder=po, deriv=1, delta=dt_medio, mode='nearest')
        label_acc = 'Accelerazione Calcolata (Derivata - Filtrata SG)'
    else:
        plot_acc = np.gradient(plot_vel, t_raw)
        label_acc = 'Accelerazione Calcolata (Raw)'

    # --- 3. PLOTTING ---
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title('Confronto: Ideale vs Esecuzione Reale')

    # Subplot 1
    axs[0].plot(t_ideal, pos_ideal, 'g-', label='Traiettoria Ideale (Ruckig)', linewidth=2)
    axs[0].plot(t_raw, plot_dist, 'ko', label=label_dist, markersize=4, alpha=0.6)
    axs[0].set_ylabel('Distanza [m]')
    axs[0].set_title('Confronto Posizione')
    axs[0].grid(True); axs[0].legend()

    # Subplot 2
    axs[1].plot(t_ideal, vel_ideal, color='orange', label='Velocità Ideale', linewidth=2)
    axs[1].plot(t_raw, plot_vel, 'ko', label=label_vel, markersize=4, alpha=0.6)
    axs[1].set_ylabel('Velocità [m/s]')
    axs[1].set_title('Confronto Velocità')
    axs[1].grid(True); axs[1].legend()

    # Subplot 3 (Corretto t_raw)
    axs[2].plot(t_ideal, acc_ideal, 'r-', label='Accelerazione Ideale', linewidth=2)
    axs[2].plot(t_raw, plot_acc, 'ko', label=label_acc, markersize=4, alpha=0.6)
    axs[2].set_ylabel('Accelerazione [m/s²]')
    axs[2].set_xlabel('Tempo [s]')
    axs[2].set_title('Confronto Accelerazione')
    axs[2].axhline(0, color='black', linewidth=0.8, linestyle=':')
    axs[2].grid(True); axs[2].legend()

    plt.tight_layout()

    # --- 4. SALVATAGGIO O VISUALIZZAZIONE ---
    if save_results:
        # Calcola i path in base a dove si trova lo script
        script_dir = Path(__file__).parent.absolute()
        results_dir = script_dir.parent / 'results'
        
        if save_subdir:
            results_dir = results_dir / save_subdir
            
        results_dir.mkdir(parents=True, exist_ok=True)
        
        # Salvataggio con il nome richiesto
        save_file = results_dir / "ruckig_vs_cartesian.png"
        
        fig.savefig(save_file, dpi=300)
        print(f"✅ Grafico salvato in: {save_file}")
        
        plt.close(fig) 
    else:
        plt.show()

if __name__ == '__main__':
    main()