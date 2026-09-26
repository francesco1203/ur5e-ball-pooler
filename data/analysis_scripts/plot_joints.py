"""
plot_joints.py

Visualizza i dati di posizione, velocità e accelerazione dei giunti, 
estraendo i dati da un bagfile
"""

import sys
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from rosbags.highlevel import AnyReader

def extract_joints_from_bag(bag_path, topic_name):
    """Estrae i dati di posizione e velocità dei giunti e i timestamp da un bag ROS 2."""
    times = []
    joint_pos = {}
    joint_vel = {}
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
                        joint_pos[name] = []
                        joint_vel[name] = []
                        
                times.append(t_sec)
                
                # Controllo di sicurezza: verifichiamo che l'array velocity esista e abbia la stessa lunghezza
                has_vel = len(msg.velocity) == len(msg.position)
                
                for i, name in enumerate(joint_names):
                    joint_pos[name].append(msg.position[i])
                    if has_vel:
                        joint_vel[name].append(msg.velocity[i])
                    else:
                        joint_vel[name].append(0.0) # Fallback se il driver non pubblica le velocità
                    
    except Exception as e:
        print(f"Errore nella lettura del bagfile {bag_path}: {e}")
        sys.exit(1)

    # Costruiamo un dizionario con colonne distinte per posizione e velocità
    data_dict = {}
    for name in joint_names:
        data_dict[f'{name}_pos'] = joint_pos[name]
        data_dict[f'{name}_vel'] = joint_vel[name]
        
    df = pd.DataFrame(data_dict)
    df['time_sec'] = times
    
    return df, joint_names

def main():

    print("Joint State Plotter - Visualizza i dati di posizione, velocità ed accelerazione per ogni giunto")

    if len(sys.argv) < 2:
        print("Uso: python3 plot_joints.py <percorso_al_bag>")
        sys.exit(1)
        
    bag_path = sys.argv[1]
    topic_target = '/joint_states'

    print(f"Estrazione dati da: {bag_path} sul topic: {topic_target}...")
    df, joint_cols = extract_joints_from_bag(bag_path, topic_target)

    if df.empty or not joint_cols:
        print("Nessun dato giunto trovato. Uscita.")
        return

    # --- CONTROLLO VELOCITÀ NULLE O MANCANTI (SIMULAZIONE MOCK) ---
    is_all_invalid = True
    for col in joint_cols:
        vel_col = f'{col}_vel'
        # Se c'è almeno un valore di velocità che NON è NaN e NON è 0.0, allora abbiamo dati reali
        if not (df[vel_col].isna() | (df[vel_col] == 0.0)).all():
            is_all_invalid = False
            break
            
    if is_all_invalid:
        print("\n" + "="*75)
        print(" ⚠️  ATTENZIONE: Le velocità dei giunti contengono solo ZERI o NaN (anche l'accelerazione, che è derivata).")
        print("     È probabile che questo bag provenga da una simulazione")
        print("     (es. mock_components) in cui la velocità non viene pubblicata.")
        print("     I grafici di velocità e accelerazione appariranno piatti.")
        print("="*75 + "\n")

    # --- 1. PULIZIA DEI TIMESTAMP ---
    df['dt'] = df['time_sec'].diff()
    df = df[(df['dt'].isna()) | (df['dt'] > 1e-3)].copy()

    # --- 2. RIMOZIONE TRANSITORIO INIZIALE (CROP) ---
    # Scartiamo i primissimi 5 campioni (circa 10-50ms) per eliminare 
    # assestamenti del controller e spike di avvio della registrazione.
    if len(df) > 10:
        df = df.iloc[5:].copy()


    # --- 2b. RIMOZIONE CODA STATICA (CROP FINALE) ---
    # Selezioniamo solo le colonne relative alle posizioni
    pos_columns = [f'{joint}_pos' for joint in joint_cols]
    
    # Calcoliamo la variazione assoluta per ogni step e prendiamo il massimo tra tutti i giunti
    max_pos_diff = df[pos_columns].diff().abs().max(axis=1).values
    
    # Troviamo gli indici in cui c'è movimento (variazione > 1e-4 rad, circa 0.005 gradi)
    active_indices = np.where(max_pos_diff > 1e-4)[0]
    
    if len(active_indices) > 0:
        last_active_idx = active_indices[-1] # L'ultimo frame in cui si muove qualcosa
        buffer_samples = 30                  # Quanti campioni (es. 30 = ~0.3 secondi a 100Hz) tenere dopo l'arresto
        
        # Tagliamo il DataFrame
        cut_idx = min(last_active_idx + buffer_samples, len(df))
        df = df.iloc[:cut_idx].copy()
        print(f"Coda statica rimossa: mantenuti {cut_idx} campioni su {len(max_pos_diff)} originali.")
    else:
        print("Nessun movimento rilevato nell'intero log.")

    # Ricalcoliamo il tempo partendo da 0 rispetto al nuovo primo campione
    df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]
    t = df['time_sec'].values


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
        raw_pos = df[f'{joint}_pos'].values
        raw_vel = df[f'{joint}_vel'].values  # <-- Ora leggiamo la velocità direttamente dal DataFrame

        # --- Filtro Posizione ---
        if filtering_position:
            # mode='nearest' previene gli sbandamenti polinomiali ai bordi
            plot_pos = savgol_filter(raw_pos, window_length=filter_sg_window_length, polyorder=filter_sg_polyorder, mode='nearest')
        else:
            plot_pos = raw_pos

        # --- Filtro Velocità ---
        if filtering_velocity:
            plot_vel = savgol_filter(raw_vel, window_length=filter_sg_window_length, polyorder=filter_sg_polyorder, mode='nearest')
        else:
            plot_vel = raw_vel

        # --- Calcolo e Filtro Accelerazione ---
        # L'accelerazione la deriviamo dalla velocità letta (e possibilmente filtrata)
        raw_acc = np.gradient(plot_vel, t)          

        if filtering_acceleration:
            plot_acc = savgol_filter(raw_acc, window_length=filter_sg_window_length, polyorder=filter_sg_polyorder, mode='nearest')
        else:
            plot_acc = raw_acc

        # Plot
        axs[0].plot(t, plot_pos, label=joint)
        axs[1].plot(t, plot_vel, label=f'vel_{joint}')
        axs[2].plot(t, plot_acc, label=f'acc_{joint}')

    # --- Estetica dei Grafici ---
    axs[0].set_ylabel('Posizione [rad]')
    if filtering_position:
        axs[0].set_title('Posizione dei Giunti nel Tempo (filtrato SG)')
    else:
        axs[0].set_title('Posizione dei Giunti nel Tempo')
    axs[0].grid(True)
    axs[0].legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))

    axs[1].set_ylabel('Velocità [rad/s]')
    if filtering_velocity:
        axs[1].set_title('Velocità dei Giunti nel Tempo (dal topic, filtrata SG)')
    else:
        axs[1].set_title('Velocità dei Giunti nel Tempo (dal topic)')
    axs[1].grid(True)
    axs[1].legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))

    axs[2].set_ylabel('Accelerazione [rad/s²]')
    axs[2].set_xlabel('Tempo [s]')
    if filtering_acceleration:
        axs[2].set_title('Accelerazione dei Giunti nel Tempo (filtrata SG)')
    else:
        axs[2].set_title('Accelerazione dei Giunti nel Tempo')
    axs[2].grid(True)
    axs[2].legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()