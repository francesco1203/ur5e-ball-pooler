import sys
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt  # <--- ASSICURATI DI AVERE QUESTA RIGA IN CIMA AL FILE
from rosbags.highlevel import AnyReader

def extract_torque_from_bag(bag_path, topic_name):
    """Estrae i dati di effort/coppia (e posizione per il crop) e i timestamp da un bag ROS 2."""
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
                        torque_data[f"{name}_pos"] = [] # <--- Aggiunto per tracciare il crop
                        
                times.append(t_sec)
                
                # SE L'EFFORT E' PRESENTE E DELLA LUNGHEZZA CORRETTA, LO USIAMO
                if hasattr(msg, 'effort') and len(msg.effort) == len(joint_names):
                    efforts = list(msg.effort)
                else:
                    efforts = [np.nan] * len(joint_names)
                    
                # ESTRAIAMO ANCHE LE POSIZIONI (servono a capire quando il braccio è fermo)
                if hasattr(msg, 'position') and len(msg.position) == len(joint_names):
                    positions = list(msg.position)
                else:
                    positions = [0.0] * len(joint_names)
                
                for i, name in enumerate(joint_names):
                    torque_data[name].append(efforts[i])
                    torque_data[f"{name}_pos"].append(positions[i])
                    
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

    print("Plot Torque - Visualizza i dati di coppia per ogni giunto.")
    
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

    # --- 1. PULIZIA DEI TIMESTAMP ---
    df['dt'] = df['time_sec'].diff()
    df = df[(df['dt'].isna()) | (df['dt'] > 1e-3)].copy()

    # --- 2. RIMOZIONE CODA STATICA ---
    # Invece del rumore della coppia, analizziamo la variazione della posizione per il taglio
    pos_columns = [f"{col}_pos" for col in joint_columns]
    
    max_pos_diff = df[pos_columns].diff().abs().max(axis=1).values
    active_indices = np.where(max_pos_diff > 1e-4)[0]
    
    if len(active_indices) > 0:
        last_active_idx = active_indices[-1]
        buffer_samples = 30  # Mantiene ~0.3s dopo l'arresto
        
        cut_idx = min(last_active_idx + buffer_samples, len(df))
        df = df.iloc[:cut_idx].copy()
        print(f"Coda statica rimossa: mantenuti {cut_idx} campioni su {len(max_pos_diff)} originali.")
    else:
        print("Nessun movimento rilevato nell'intero log (posizioni costanti o assenti).")

    # Ricalcolo il tempo partendo da 0
    df['time_sec'] = df['time_sec'] - df['time_sec'].iloc[0]

    # --- PLOTTING ---
    n_joints = len(joint_columns)
    fig, axes = plt.subplots(n_joints, 1, figsize=(10, 2.5 * n_joints), sharex=True)
    fig.canvas.manager.set_window_title(f'Analisi Coppie Giunti - {bag_path}')

    if n_joints == 1:
        axes = [axes]

    for ax, joint_name in zip(axes, joint_columns):
        # Utilizziamo la colonna dell'effort per il plot (non la posizione!)
        ax.plot(df["time_sec"], df[joint_name], linewidth=1.5, color='#1f77b4')
        ax.set_ylabel("Effort [Nm]")
        ax.set_title(joint_name)
        ax.grid(True, alpha=0.4)

    axes[-1].set_xlabel("Tempo [s]")
    fig.tight_layout()

    plt.show()