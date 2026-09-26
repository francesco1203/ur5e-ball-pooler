"""
cartesian_vs_ruckig.py

Visualizza i dati di posizione, velocità e accelerazione cartesiana in linea retta, 
estraendo i dati da un bagfile, confrontando con i dati ideali generati da Ruckig.
"""

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter  # IMPORTANTE: Aggiunto per il filtro SG

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
        
    # Salva le classi dei messaggi per deserializzare correttamente i vari topic
    msg_types = {
        pose_topic: get_message(type_map[pose_topic]),
        twist_topic: get_message(type_map[twist_topic])
    }

    # Filtra entrambi i topic
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

    # Crea due dataframe e ordinali per tempo
    df_pose = pd.DataFrame({'time_sec': pose_times, 'x': xs, 'y': ys, 'z': zs}).sort_values('time_sec')
    df_twist = pd.DataFrame({'time_sec': twist_times, 'vx': vxs, 'vy': vys, 'vz': vzs}).sort_values('time_sec')
    
    if df_pose.empty or df_twist.empty:
        return pd.DataFrame()

    # Unisci i dati allineando il twist al timestamp della pose più vicina
    df = pd.merge_asof(df_pose, df_twist, on='time_sec', direction='nearest')
    
    if not df.empty:
        df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]
        
    return df

def main():

    print("Cartesian vs Ruckig: Confronto tra traiettoria ideale e reale")


    # Se passi i percorsi da terminale: python script.py <file_csv_ideale> <cartella_bag_raw>
    if len(sys.argv) < 3:
        print("Uso: python3 compare_ruckig.py <file_ideale.csv> <cartella_bag_reale>")
        sys.exit(1)

    ideal_file_path = sys.argv[1]
    raw_bag_path = sys.argv[2]
    topic_pose = '/tcp_pose_broadcaster/pose'
    topic_twist = '/tcp_twist'

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
    df_raw = extract_kinematics_from_bag(raw_bag_path, topic_pose, topic_twist)

    if df_raw.empty:
        print("Nessun dato estratto dal bag. Uscita.")
        return

    # --- INIZIO NUOVO BLOCCO CONTROLLO VELOCITÀ CARTESIANE ---
    is_all_invalid = True
    for col in ['vx', 'vy', 'vz']:
        if not (df_raw[col].isna() | (df_raw[col] == 0.0)).all():
            is_all_invalid = False
            break
            
    if is_all_invalid:
        print("\n" + "="*75)
        print(" ⚠️  ATTENZIONE: Le velocità reali (Twist) contengono solo ZERI o NaN.")
        print("     È probabile che questo bag provenga da una simulazione (es. mock_components)")
        print("     dove il Cartesian Velocity Publisher non riceve le velocità dei giunti.")
        print("     I plot di confronto mostreranno i dati reali di velocità/accelerazione piatti.")
        print("="*75 + "\n")
    # --- FINE NUOVO BLOCCO ---

    
    # Pulizia timestamp per evitare divisioni per zero o picchi irreali (burst initiali)
    df_raw['dt'] = df_raw['time_sec'].diff()
    df_raw = df_raw[(df_raw['dt'].isna()) | (df_raw['dt'] > 1e-3)].copy()
    df_raw['time_sec'] = df_raw['time_sec'] - df_raw['time_sec'].iloc[0]

    # --- NUOVO BLOCCO: RIMOZIONE CODA STATICA (Sui dati reali) ---
    temp_x = df_raw['x'].values
    temp_y = df_raw['y'].values
    temp_z = df_raw['z'].values
    
    # Calcolo della distanza radiale dal punto di partenza
    temp_dist = np.sqrt((temp_x - temp_x[0])**2 + (temp_y - temp_y[0])**2 + (temp_z - temp_z[0])**2)
    dist_diff = np.abs(np.diff(temp_dist, prepend=0.0))
    
    # Cerchiamo gli indici dove l'End-Effector si sta muovendo (soglia ~0.1 mm)
    active_indices = np.where(dist_diff > 1e-4)[0]
    
    if len(active_indices) > 0:
        last_active_idx = active_indices[-1]
        buffer_samples = 10  # Mantiene ~0.3 secondi dopo l'arresto
        
        cut_idx = min(last_active_idx + buffer_samples, len(df_raw))
        df_raw = df_raw.iloc[:cut_idx].copy()
        print(f"Coda statica rimossa (dati reali): mantenuti {cut_idx} campioni su {len(dist_diff)} originali.")
    else:
        print("Nessun movimento cartesiano rilevato nell'intero log reale.")
    # --- FINE RIMOZIONE CODA STATICA ---

    # Ricalcolo il tempo partendo da 0 sul DataFrame pulito e tagliato
    df_raw['time_sec'] = df_raw['time_sec'] - df_raw['time_sec'].iloc[0]

    t_raw = df_raw['time_sec'].values
    x, y, z = df_raw['x'].values, df_raw['y'].values, df_raw['z'].values
    vx, vy, vz = df_raw['vx'].values, df_raw['vy'].values, df_raw['vz'].values

    # Calcoli grezzi
    dist_raw = np.sqrt((x - x[0])**2 + (y - y[0])**2 + (z - z[0])**2)
    vel_raw = np.sqrt(vx**2 + vy**2 + vz**2)

    # --- FILTRAGGIO SAVITZKY-GOLAY (Opzionale) ---
    filtering_distance = False
    filtering_velocity = False
    filtering_acceleration = False
    
    wl = 9  # Window length (deve essere dispari)
    po = 3  # Polynomial order

    # Filtra (o mantieni grezza) la distanza
    plot_dist = savgol_filter(dist_raw, window_length=wl, polyorder=po) if filtering_distance else dist_raw
    label_dist = 'Campioni Reali (Filtrati SG)' if filtering_distance else 'Campioni Reali (Raw)'

    # Filtra (o mantieni grezza) la velocità
    plot_vel = savgol_filter(vel_raw, window_length=wl, polyorder=po) if filtering_velocity else vel_raw
    label_vel = 'Velocità Calcolata (Filtrata SG)' if filtering_velocity else 'Velocità Calcolata (Raw)'

    # Calcolo dell'accelerazione basato sulla velocità (filtrata o meno)
    dt_vel = np.diff(t_raw)
    acc_raw = np.diff(plot_vel) / dt_vel
    t_acc_raw = t_raw[:-1]

    # Filtra (o mantieni grezza) l'accelerazione
    plot_acc = savgol_filter(acc_raw, window_length=wl, polyorder=po) if filtering_acceleration else acc_raw
    label_acc = 'Accelerazione Calcolata (Filtrata SG)' if filtering_acceleration else 'Accelerazione Calcolata (Raw)'

    # --- 3. PLOTTING SOVRAPPOSTO ---
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title('Confronto: Ideale vs Esecuzione Reale')

    # Subplot 1: Posizione
    axs[0].plot(t_ideal, pos_ideal, 'g-', label='Traiettoria Ideale (Ruckig)', linewidth=2)
    axs[0].plot(t_raw, plot_dist, 'ko', label=label_dist, markersize=4, alpha=0.6)
    axs[0].set_ylabel('Distanza [m]')
    axs[0].set_title('Confronto Posizione')
    axs[0].grid(True); axs[0].legend()

    # Subplot 2: Velocità
    axs[1].plot(t_ideal, vel_ideal, color='orange', label='Velocità Ideale', linewidth=2)
    axs[1].plot(t_raw, plot_vel, 'ko', label=label_vel, markersize=4, alpha=0.6)
    axs[1].set_ylabel('Velocità [m/s]')
    axs[1].set_title('Confronto Velocità')
    axs[1].grid(True); axs[1].legend()

    # Subplot 3: Accelerazione
    axs[2].plot(t_ideal, acc_ideal, 'r-', label='Accelerazione Ideale', linewidth=2)
    axs[2].plot(t_acc_raw, plot_acc, 'ko', label=label_acc, markersize=4, alpha=0.6)
    axs[2].set_ylabel('Accelerazione [m/s²]')
    axs[2].set_xlabel('Tempo [s]')
    axs[2].set_title('Confronto Accelerazione')
    axs[2].axhline(0, color='black', linewidth=0.8, linestyle=':')
    axs[2].grid(True); axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()