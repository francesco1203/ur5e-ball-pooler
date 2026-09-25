"""
plot_cartesian.py

Visualizza i dati di posizione, velocità e accelerazione cartesiana in linea retta, 
estraendo i dati da un bagfile
"""


import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

from rosbags.highlevel import AnyReader
from pathlib import Path

def extract_kinematics_from_bag(bag_path, pose_topic, twist_topic):
    """Estrae i dati di posizione e twist cartesiano allineandoli temporalmente."""
    pose_times, xs, ys, zs = [], [], [], []
    twist_times, vxs, vys, vzs = [], [], [], []
    
    try:
        bagpath = Path(bag_path)
        
        with AnyReader([bagpath]) as reader:
            target_topics = [pose_topic, twist_topic]
            topic_connections = [conn for conn in reader.connections if conn.topic in target_topics]
            
            if not topic_connections:
                print(f"Errore: Nessuno dei topic richiesti è nel bagfile.")
                sys.exit(1)

            for connection, timestamp, rawdata in reader.messages(connections=topic_connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                t_sec = timestamp / 1e9 
                
                if connection.topic == pose_topic:
                    xs.append(msg.pose.position.x)
                    ys.append(msg.pose.position.y)
                    zs.append(msg.pose.position.z)
                    pose_times.append(t_sec)
                    
                elif connection.topic == twist_topic:
                    vxs.append(msg.twist.linear.x)
                    vys.append(msg.twist.linear.y)
                    vzs.append(msg.twist.linear.z)
                    twist_times.append(t_sec)
                
    except Exception as e:
        print(f"Errore nella lettura del bagfile {bag_path}: {e}")
        sys.exit(1)

    # Creiamo due dataframe separati
    df_pose = pd.DataFrame({'time_sec': pose_times, 'x': xs, 'y': ys, 'z': zs}).sort_values('time_sec')
    df_twist = pd.DataFrame({'time_sec': twist_times, 'vx': vxs, 'vy': vys, 'vz': vzs}).sort_values('time_sec')
    
    if df_pose.empty or df_twist.empty:
        return pd.DataFrame()

    # Uniamo i due dataframe allineando il twist al timestamp della pose più vicina
    df = pd.merge_asof(df_pose, df_twist, on='time_sec', direction='nearest')
    
    # Normalizza il tempo in modo che parta da 0
    df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]
        
    return df

def main():
    # 1. LETTURA DATI DAL BAGFILE ROS 2
    if len(sys.argv) < 2:
        print("Uso: python script.py <percorso_al_bag>")
        return

    bag_path = sys.argv[1]
    
    # --- ASSICURATI CHE QUESTI NOMI SIANO CORRETTI PER IL TUO SISTEMA ---
    topic_pose = '/tcp_pose_broadcaster/pose'
    topic_twist = '/tcp_twist' 

    print(f"Estrazione dati da: {bag_path}...")
    df = extract_kinematics_from_bag(bag_path, topic_pose, topic_twist)

    if df.empty:
        print("Nessun dato estratto. Verifica i nomi dei topic. Uscita.")
        return
        
    print(f"Estratti {len(df)} messaggi combinati. Pulizia timestamp...")

    # Pulizia dei timestamp
    df['dt'] = df['time_sec'].diff()
    df = df[(df['dt'].isna()) | (df['dt'] > 1e-3)].copy()
    df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]

    # Estrazione array numpy
    t = df['time_sec'].values
    x, y, z = df['x'].values, df['y'].values, df['z'].values
    vx, vy, vz = df['vx'].values, df['vy'].values, df['vz'].values

    # 2. CALCOLO DI DISTANZA, VELOCITÀ E ACCELERAZIONE CARTESIANE
    filtering_distance = True
    filtering_velocity = True
    filtering_acceleration = True
    
    wl = 9 # window_length (deve essere dispari)
    po = 3 # polyorder

    # Distanza percorsa (grezza per il filtro, ma non la deriviamo più!)
    raw_dist = np.sqrt((x - x[0])**2 + (y - y[0])**2 + (z - z[0])**2)
    plot_dist = savgol_filter(raw_dist, window_length=wl, polyorder=po) if filtering_distance else raw_dist

    # --- NUOVO CALCOLO VELOCITÀ: Modulo del Twist Lineare ---
    raw_vel = np.sqrt(vx**2 + vy**2 + vz**2)
    vel_cart = savgol_filter(raw_vel, window_length=wl, polyorder=po) if filtering_velocity else raw_vel

    # Accelerazione rimane derivata della velocità
    acc_cart = np.gradient(vel_cart, t)
    if filtering_acceleration:
        acc_cart = savgol_filter(acc_cart, window_length=wl, polyorder=po)

    # 3. PLOTTING DEI RISULTATI
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title(f'Analisi Cartesiana - {bag_path}')

    # Subplot 1: Distanza
    axs[0].plot(t, plot_dist, 'g-', label='Distanza percorsa [m]')
    axs[0].set_ylabel('Distanza [m]')
    axs[0].set_title('Profilo di Distanza Cartesiana' + (' (filtrata SG)' if filtering_distance else ''))
    axs[0].grid(True)
    axs[0].legend()

    # Subplot 2: Velocità
    axs[1].plot(t, vel_cart, color='orange', label='Velocità cartesiana [m/s]')
    axs[1].set_ylabel('Velocità [m/s]')
    axs[1].set_title('Profilo di Velocità Cartesiana' + (' (filtrata SG)' if filtering_velocity else ''))
    axs[1].grid(True)
    axs[1].legend()

    # Subplot 3: Accelerazione
    axs[2].plot(t, acc_cart, color='red', label='Accelerazione cartesiana [m/s²]')
    axs[2].set_ylabel('Accelerazione [m/s²]')
    axs[2].set_xlabel('Tempo [s]')
    axs[2].set_title('Profilo di Accelerazione Cartesiana' + (' (filtrata SG)' if filtering_acceleration else ''))
    axs[2].axhline(0, color='black', linewidth=0.8, linestyle=':')
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()