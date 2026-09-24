import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

from rosbags.highlevel import AnyReader
from pathlib import Path

def extract_pose_from_bag(bag_path, topic_name):
    """Estrae i dati di posizione cartesiana e i timestamp da un bag ROS 2."""
    times, xs, ys, zs = [], [], [], []
    
    try:
        # Convertiamo il percorso stringa in un oggetto Path
        bagpath = Path(bag_path)
        
        # Inizializziamo AnyReader passando il percorso in una lista
        with AnyReader([bagpath]) as reader:
            # Verifica se il topic esiste nel bag
            topic_connections = [conn for conn in reader.connections if conn.topic == topic_name]
            if not topic_connections:
                print(f"Errore: Il topic '{topic_name}' non è presente nel bagfile.")
                sys.exit(1)

            # Legge i messaggi del topic
            for connection, timestamp, rawdata in reader.messages(connections=topic_connections):
                
                # LA NOVITÀ: Deserializza usando direttamente l'oggetto reader!
                msg = reader.deserialize(rawdata, connection.msgtype)
                
                # Il timestamp di rosbags è in nanosecondi (epoch). Lo convertiamo in secondi.
                t_sec = timestamp / 1e9 
                
                # Assumiamo che il tipo di messaggio sia geometry_msgs/msg/PoseStamped
                xs.append(msg.pose.position.x)
                ys.append(msg.pose.position.y)
                zs.append(msg.pose.position.z)
                times.append(t_sec)
                
    except Exception as e:
        print(f"Errore nella lettura del bagfile {bag_path}: {e}")
        sys.exit(1)

    # Creazione DataFrame
    df = pd.DataFrame({'time_sec': times, 'x': xs, 'y': ys, 'z': zs})
    
    # Normalizza il tempo in modo che parta da 0 per facilitare la lettura del grafico
    if not df.empty:
        df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]
        
    return df
def main():
    # 1. LETTURA DATI DAL BAGFILE ROS 2
    bag_path = sys.argv[1]
    topic_target = '/tcp_pose_broadcaster/pose'

    print(f"Estrazione dati da: {bag_path} sul topic: {topic_target}...")
    df = extract_pose_from_bag(bag_path, topic_target)

    if df.empty:
        print("Nessun dato estratto. Uscita.")
        return
        
    print(f"Estratti {len(df)} messaggi. Pulizia timestamp...")

    # --- MODIFICA 1: PULIZIA DEI TIMESTAMP ---
    # Calcola il delta temporale tra ogni campione
    df['dt'] = df['time_sec'].diff()
    
    # Tieni solo la prima riga (che ha dt=NaN) oppure le righe dove il tempo 
    # è avanzato di almeno 1 millisecondo (0.001 s). 
    # Questo elimina i "burst" di messaggi pubblicati nello stesso istante.
    df = df[(df['dt'].isna()) | (df['dt'] > 1e-3)].copy()
    
    # Ricalcola il tempo da 0 dopo la pulizia
    df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]

    # Estrazione array numpy dal DataFrame pulito
    t = df['time_sec'].values
    x = df['x'].values
    y = df['y'].values
    z = df['z'].values

    # 2. CALCOLO DI DISTANZA, VELOCITÀ E ACCELERAZIONE CARTESIANE
    filtering_distance = True
    filtering_velocity = True
    filtering_acceleration = True
    
    # Finestra e ordine per il filtro (puoi regolarli se serve)
    wl = 9 # window_length (deve essere dispari)
    po = 3  # polyorder

    # Distanza percorsa dall'inizio (Lasciala grezza per il calcolo della velocità!)
    raw_dist = np.sqrt((x - x[0])**2 + (y - y[0])**2 + (z - z[0])**2)

    # Filtra la distanza SOLO per il plot, non usare quella filtrata per le derivate
    if filtering_distance:
        plot_dist = savgol_filter(raw_dist, window_length=wl, polyorder=po)
    else:
        plot_dist = raw_dist

    # --- MODIFICA 2: DERIVA PRIMA, FILTRA DOPO ---
    # Derivata numerica per Velocità (sulla distanza grezza, non influenzata dai bordi SG)
    vel_cart = np.gradient(raw_dist, t)

    # Filtro su Velocità
    if filtering_velocity:
        vel_cart = savgol_filter(vel_cart, window_length=wl, polyorder=po)

    # Derivata numerica per Accelerazione (sulla velocità filtrata)
    acc_cart = np.gradient(vel_cart, t)

    # Filtro su Accelerazione
    if filtering_acceleration:
        acc_cart = savgol_filter(acc_cart, window_length=wl, polyorder=po)

    # 3. PLOTTING DEI RISULTATI
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title(f'Analisi Cartesiana - {bag_path}')

    # Subplot 1: Distanza (usiamo plot_dist)
    axs[0].plot(t, plot_dist, 'g-', label='Distanza percorsa [m]')
    axs[0].set_ylabel('Distanza [m]')
    if filtering_distance:
        axs[0].set_title('Profilo di Distanza Cartesiana (filtrata SG)')
    else:
        axs[0].set_title('Profilo di Distanza Cartesiana')
    
    axs[0].grid(True)
    axs[0].legend()

    # Subplot 2: Velocità
    axs[1].plot(t, vel_cart, color='orange', label='Velocità cartesiana [m/s]')
    axs[1].set_ylabel('Velocità [m/s]')
    if filtering_velocity:
        axs[1].set_title('Profilo di Velocità Cartesiana (filtrata SG)')
    else:
        axs[1].set_title('Profilo di Velocità Cartesiana')  
    axs[1].grid(True)
    axs[1].legend()

    # Subplot 3: Accelerazione
    axs[2].plot(t, acc_cart, color='red', label='Accelerazione cartesiana [m/s²]')
    axs[2].set_ylabel('Accelerazione [m/s²]')
    axs[2].set_xlabel('Tempo [s]')
    if filtering_acceleration:
        axs[2].set_title('Profilo di Accelerazione Cartesiana (filtrata SG)')
    else:
        axs[2].set_title('Profilo di Accelerazione Cartesiana') 
    axs[2].axhline(0, color='black', linewidth=0.8, linestyle=':')
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()