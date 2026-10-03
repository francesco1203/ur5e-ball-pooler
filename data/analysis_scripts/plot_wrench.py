"""
plot_wrench.py

Visualizza i dati di forza (Force) e coppia (Torque) misurati dal sensore, 
estraendo i dati raw e filtrati da un bagfile ROS 2.
Supporta il salvataggio automatico in una sottocartella in data/results 
se viene passato il flag --save <nome_cartella>.
"""

import sys
import os
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from rosbags.highlevel import AnyReader

def extract_wrench_from_bag(bag_path, topic_name):
    """Estrae i dati Wrench (forza e coppia) e i timestamp da un bag ROS 2."""
    times = []
    data = {'fx': [], 'fy': [], 'fz': [], 'tx': [], 'ty': [], 'tz': []}
    
    try:
        bagpath = Path(bag_path)
        with AnyReader([bagpath]) as reader:
            topic_connections = [conn for conn in reader.connections if conn.topic == topic_name]
            if not topic_connections:
                return pd.DataFrame()  # Ritorna DataFrame vuoto se il topic non esiste

            for connection, timestamp, rawdata in reader.messages(connections=topic_connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                
                # Usiamo il timestamp dell'header del messaggio per massima precisione
                t_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
                
                times.append(t_sec)
                data['fx'].append(msg.wrench.force.x)
                data['fy'].append(msg.wrench.force.y)
                data['fz'].append(msg.wrench.force.z)
                data['tx'].append(msg.wrench.torque.x)
                data['ty'].append(msg.wrench.torque.y)
                data['tz'].append(msg.wrench.torque.z)
                    
    except Exception as e:
        print(f"Errore nella lettura del bagfile {bag_path}: {e}")
        sys.exit(1)

    df = pd.DataFrame(data)
    df['time_sec'] = times
    
    return df

def process_dataframe(df, min_time, is_raw=False):
    """Applica pulizia, crop iniziale e calcolo del tempo relativo al DataFrame."""
    if df.empty:
        return df

    # --- 1. PULIZIA DEI TIMESTAMP ---
    df['dt'] = df['time_sec'].diff()
    df = df[(df['dt'].isna()) | (df['dt'] > 1e-4)].copy()

    # --- 2. RIMOZIONE TRANSITORIO INIZIALE (CROP) ---
    if len(df) > 10:
        df = df.iloc[5:].copy()

    # Sincronizziamo il tempo con il t0 globale del bag
    df['time_sec'] = df['time_sec'] - min_time

    return df

def get_static_tail_cut_index(df, threshold=0.5, buffer_samples=50):
    """Calcola l'indice a cui tagliare la coda statica basandosi sui picchi di forza."""
    if df.empty:
        return None
    
    # Calcoliamo la variazione assoluta per ogni step per le forze
    max_force_diff = df[['fx', 'fy', 'fz']].diff().abs().max(axis=1).values
    
    # Troviamo gli indici in cui c'è movimento (variazione di forza > threshold)
    active_indices = np.where(max_force_diff > threshold)[0]
    
    if len(active_indices) > 0:
        last_active_idx = active_indices[-1]
        cut_idx = min(last_active_idx + buffer_samples, len(df))
        return cut_idx
    return None

def main():
    print("Wrench Data Plotter - Visualizza le forze e le coppie scambiate all'end-effector")

    # --- Lettura Argomenti, Flag e Sottocartella ---
    args = sys.argv[1:]
    if len(args) < 1:
        print("Uso: python3 plot_wrench.py <percorso_al_bag> [--save <sottocartella>]")
        sys.exit(1)
        
    save_results = '--save' in args
    save_subdir = None
    
    if save_results:
        idx = args.index('--save')
        # Verifica se l'utente ha passato il nome della sottocartella subito dopo --save
        if idx + 1 < len(args) and not args[idx+1].startswith('-'):
            save_subdir = args[idx + 1]
            args.pop(idx + 1) # Rimuove la stringa della sottocartella dagli argomenti
        args.pop(idx) # Rimuove '--save' dagli argomenti
        
    if len(args) == 0:
        print("Errore: Manca il percorso al bagfile.")
        sys.exit(1)
        
    bag_path = args[0]
    topic_raw = '/force_torque_sensor_broadcaster/wrench'
    topic_filtered = '/force_torque_sensor_broadcaster/wrench_filtered'

    print(f"Estrazione dati da: {bag_path}...")
    
    df_raw = extract_wrench_from_bag(bag_path, topic_raw)
    df_filt = extract_wrench_from_bag(bag_path, topic_filtered)

    if df_raw.empty and df_filt.empty:
        print("Nessun dato wrench (né raw né filtered) trovato nel bagfile. Uscita.")
        return

    # --- CONTROLLO DATI MANCANTI O NULLI (SIMULAZIONE) ---
    is_all_invalid = True
    wrench_cols = ['fx', 'fy', 'fz', 'tx', 'ty', 'tz']
    
    # Controlliamo il dataframe raw (se esiste)
    if not df_raw.empty:
        for col in wrench_cols:
            if not (df_raw[col].isna() | (df_raw[col] == 0.0)).all():
                is_all_invalid = False
                break
                
    # Se il raw è tutto nullo/vuoto, controlliamo anche il filtrato (se esiste)
    if is_all_invalid and not df_filt.empty:
        for col in wrench_cols:
            if not (df_filt[col].isna() | (df_filt[col] == 0.0)).all():
                is_all_invalid = False
                break

    if is_all_invalid:
        print("\n" + "="*75)
        print(" ⚠️  ATTENZIONE: I dati Wrench (Force/Torque) contengono solo ZERI o NaN.")
        print("     È probabile che questo bag provenga da una simulazione in cui il")
        print("     sensore di forza non è modellato fisicamente o restituisce array vuoti.")
        print("     I grafici verranno aperti, ma appariranno completamente piatti.")
        print("="*75 + "\n")
        
    # --- TROVIAMO IL TEMPO DI PARTENZA GLOBALE PER SINCRONIZZARE I TOPIC ---
    t0_raw = df_raw['time_sec'].iloc[0] if not df_raw.empty else float('inf')
    t0_filt = df_filt['time_sec'].iloc[0] if not df_filt.empty else float('inf')
    t0_global = min(t0_raw, t0_filt)

    df_raw = process_dataframe(df_raw, t0_global, is_raw=True)
    df_filt = process_dataframe(df_filt, t0_global, is_raw=False)

    # --- RIMOZIONE CODA STATICA (CROP FINALE) ---
    # Usiamo il topic filtrato (se esiste) per capire quando il robot si è fermato, essendo meno rumoroso
    reference_df = df_filt if not df_filt.empty else df_raw
    cut_idx = get_static_tail_cut_index(reference_df, threshold=0.5, buffer_samples=100) # 100 samples buffer post-impatto
    
    if cut_idx is not None:
        # Troviamo a quale istante temporale corrisponde il taglio
        cut_time = reference_df['time_sec'].iloc[cut_idx - 1]
        
        # Tagliamo entrambi i dataframe allo stesso istante temporale
        if not df_raw.empty:
            df_raw = df_raw[df_raw['time_sec'] <= cut_time].copy()
        if not df_filt.empty:
            df_filt = df_filt[df_filt['time_sec'] <= cut_time].copy()
        print(f"Coda statica rimossa: log tagliato a t = {cut_time:.2f} s")
    elif not is_all_invalid:
        print("Nessun picco di forza rilevato (tiro a vuoto o soglia troppo alta).")

    # --- CONFIGURAZIONE PLOT E FILTRI SOFTWARE AGGIUNTIVI ---
    apply_extra_sg_filter = False
    sg_window, sg_order = 11, 3

    fig, axs = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    fig.canvas.manager.set_window_title(f'Analisi Force/Torque - {bag_path}')

    axes_labels = ['x', 'y', 'z']
    colors = {'x': 'r', 'y': 'g', 'z': 'b'}

    for axis in axes_labels:
        col_f = f'f{axis}'
        col_t = f't{axis}'

        # PLOT DATI RAW (linee tratteggiate, semitrasparenti)
        if not df_raw.empty:
            t_raw = df_raw['time_sec'].values
            f_raw = df_raw[col_f].values
            tq_raw = df_raw[col_t].values
            
            axs[0].plot(t_raw, f_raw, color=colors[axis], linestyle='--', alpha=0.3, label=f'Raw F{axis}')
            axs[1].plot(t_raw, tq_raw, color=colors[axis], linestyle='--', alpha=0.3, label=f'Raw T{axis}')

        # PLOT DATI FILTRATI DAL CONTROLLER (linee continue)
        if not df_filt.empty:
            t_filt = df_filt['time_sec'].values
            f_filt = df_filt[col_f].values
            tq_filt = df_filt[col_t].values

            if apply_extra_sg_filter:
                f_filt = savgol_filter(f_filt, window_length=sg_window, polyorder=sg_order, mode='nearest')
                tq_filt = savgol_filter(tq_filt, window_length=sg_window, polyorder=sg_order, mode='nearest')

            axs[0].plot(t_filt, f_filt, color=colors[axis], linestyle='-', linewidth=2, label=f'Filt F{axis}')
            axs[1].plot(t_filt, tq_filt, color=colors[axis], linestyle='-', linewidth=2, label=f'Filt T{axis}')


    # --- Estetica dei Grafici ---
    # FORZE
    axs[0].set_ylabel('Forza [N]')
    axs[0].set_title('Forze Scambiate (Raw vs Filtered)')
    axs[0].grid(True)
    handles, labels = axs[0].get_legend_handles_labels()
    axs[0].legend(handles, labels, loc='upper left', bbox_to_anchor=(1.01, 1.0))

    # COPPIE
    axs[1].set_ylabel('Coppia [Nm]')
    axs[1].set_xlabel('Tempo [s]')
    axs[1].set_title('Coppie Scambiate (Raw vs Filtered)')
    axs[1].grid(True)
    handles, labels = axs[1].get_legend_handles_labels()
    axs[1].legend(handles, labels, loc='upper left', bbox_to_anchor=(1.01, 1.0))

    plt.tight_layout()

    # --- SALVATAGGIO O VISUALIZZAZIONE ---
    if save_results:
        # script_dir è data/analysis_scripts
        script_dir = Path(__file__).parent.absolute()
        
        # results_dir è data/results
        results_dir = script_dir.parent / 'results'
        
        if save_subdir:
            results_dir = results_dir / save_subdir
            
        results_dir.mkdir(parents=True, exist_ok=True)
        
        # Salvataggio con il nome fisso richiesto
        save_file = results_dir / "wrench.png"
        
        fig.savefig(save_file, dpi=300)
        print(f"✅ Grafico salvato in: {save_file}")
        
        plt.close(fig) 
    else:
        plt.show()

if __name__ == '__main__':
    main()