#!/usr/bin/env python3
"""
plot_controller_position.py

Visualizza gli errori di posizione (e opzionalmente desired vs actual) per ogni giunto
estraendo i dati dal bagfile usando le librerie native ROS 2 (rosbag2_py).
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt

# --- Import nativi di ROS 2 ---
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def extract_controller_state(bag_path, topic_name):
    """Estrae i dati del controller usando rosbag2_py."""
    times = []
    data = {}
    joint_names_raw = []
    
    # 1. Configurazione del lettore nativo
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

    # 2. Ottieni il tipo di messaggio corretto per il topic
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    if topic_name not in type_map:
        print(f"Errore: Il topic '{topic_name}' non è presente nel bagfile.")
        sys.exit(1)
        
    msg_type_str = type_map[topic_name]
    msg_type = get_message(msg_type_str)

    # 3. Filtra solo il topic che ci interessa per velocizzare la lettura
    storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
    reader.set_filter(storage_filter)

    # 4. Lettura dei messaggi
    while reader.has_next():
        topic, rawdata, timestamp = reader.read_next()
        
        # Deserializzazione nativa (non fallirà mai sulle dimensioni)
        msg = deserialize_message(rawdata, msg_type)
        t_sec = timestamp / 1e9 
        
        # Inizializza le chiavi del dizionario al primo messaggio
        if not joint_names_raw:
            joint_names_raw = msg.joint_names 
            for name in joint_names_raw:
                data[f"{name}_desired_pos"] = []
                data[f"{name}_actual_pos"] = []
                data[f"{name}_error_pos"] = []
                
        times.append(t_sec)
        
        # Estrai i valori
        for i, name in enumerate(joint_names_raw):
            # Nelle versioni recenti di ROS 2 (Humble+) si chiamano reference e feedback
            ref_val = msg.reference.positions[i]
            feed_val = msg.feedback.positions[i]
            
            data[f"{name}_desired_pos"].append(ref_val)
            data[f"{name}_actual_pos"].append(feed_val)
            
            if len(msg.error.positions) > i:
                error_val = msg.error.positions[i]
            else:
                error_val = ref_val - feed_val
                
            data[f"{name}_error_pos"].append(error_val)

    # Crea il DataFrame
    df = pd.DataFrame(data)
    df['time_sec'] = times
    
    # Normalizza il tempo facendolo partire da 0
    if not df.empty:
        df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]
        
    return df


def get_joint_names(columns):
    """Ricava i nomi dei giunti dalle colonne cercando il suffisso '_desired_pos'."""
    joints = []
    for col in columns:
        if col.endswith("_desired_pos"):
            joint_name = col[: -len("_desired_pos")]

            if joint_name.startswith(""):
                joint_name = joint_name[len(""):]
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
 
        ax_err.plot(df["time_sec"], df[f"{joint}_joint_error_pos"], color="crimson", linewidth=1)
        ax_err.axhline(0.0, color="black", linewidth=0.8, linestyle="--")
        ax_err.set_ylabel(f"{joint}\n")
        ax_err.grid(True, alpha=0.3)
 
        ax_cmp.plot(df["time_sec"], df[f"{joint}_joint_desired_pos"], label="Desired", color="green", linewidth=1.5)
        ax_cmp.plot(df["time_sec"], df[f"{joint}_joint_actual_pos"], label="Actual", color="black", linewidth=1, alpha=0.7)
        ax_cmp.grid(True, alpha=0.3)
 
        if i == 0:
            ax_err.set_title("Errore di posizione [rad]")
            ax_cmp.set_title("Desired position vs Actual position [rad]")
            ax_cmp.legend(loc="best")
 
    axes[-1][0].set_xlabel("Tempo [s]")
    axes[-1][1].set_xlabel("Tempo [s]")
 
    fig.tight_layout(rect=[0, 0, 1, 0.97])


def main():

    bag_path = sys.argv[1]
    topic_target = '/scaled_joint_trajectory_controller/controller_state'

    print(f"Estrazione dati controller da: {bag_path} sul topic: {topic_target}...")
    df = extract_controller_state(bag_path, topic_target)

    if df.empty:
        print("Errore: Nessun dato estratto dal controller.")
        sys.exit(1)

    joints = get_joint_names(df.columns)
    if not joints:
        print("Errore: nessuna colonna elaborata. Controlla la logica dei nomi.")
        sys.exit(1)

    print(f"Trovati {len(joints)} giunti: {joints}")
    print(f"Numero di campioni: {len(df)}")
    print(f"Durata registrata: {df['time_sec'].iloc[-1]:.3f} s")

    plot_position(df, joints)
    plt.show()

if __name__ == "__main__":
    main()