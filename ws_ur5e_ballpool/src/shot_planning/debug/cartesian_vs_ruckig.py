import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def main():
    # Percorsi dei file (assicurati che corrispondano ai tuoi percorsi reali)
    ideal_file_path = 'src/shot_planning/debug/ruckig_logging/ruckig_trajectory_log.csv'
    raw_file_path = 'src/execution_monitoring/csv_subscribers_nodes/data/cartesian_logging/cartesian_log_4_shot.csv'

    # Se passi i percorsi da terminale: python script.py <file_ideale> <file_raw>
    if len(sys.argv) > 2:
        ideal_file_path = sys.argv[1]
        raw_file_path = sys.argv[2]

    try:
        df_ideal = pd.read_csv(ideal_file_path)
        df_raw = pd.read_csv(raw_file_path)
    except Exception as e:
        print(f"Errore nella lettura dei file: {e}")
        return

    # --- 1. DATI IDEALI (RUCKIG) ---
    # Presumendo che le colonne siano: Time_s, Position_m, Velocity_ms, Acceleration_ms2
    t_ideal = df_ideal.iloc[:, 0].values
    pos_ideal = df_ideal.iloc[:, 1].values
    vel_ideal = df_ideal.iloc[:, 2].values
    acc_ideal = df_ideal.iloc[:, 3].values

    # --- 2. DATI ESECUZIONE (RAW) ---
    t_raw = df_raw['time_sec'].values
    x, y, z = df_raw['x'].values, df_raw['y'].values, df_raw['z'].values

    # Distanza reale
    dist_raw = np.sqrt((x - x[0])**2 + (y - y[0])**2 + (z - z[0])**2)

    # Velocità pura (Delta Spazio / Delta Tempo)
    dt = np.diff(t_raw)
    dt[dt == 0] = 1e-6 # Evita divisioni per zero
    vel_raw = np.diff(dist_raw) / dt
    t_vel_raw = t_raw[:-1]

    # Accelerazione pura
    dt_vel = np.diff(t_vel_raw)
    dt_vel[dt_vel == 0] = 1e-6
    acc_raw = np.diff(vel_raw) / dt_vel
    t_acc_raw = t_vel_raw[:-1]

    # --- 3. PLOTTING SOVRAPPOSTO ---
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title('Confronto: Ideale vs Esecuzione Reale')

    # Subplot 1: Posizione
    axs[0].plot(t_ideal, pos_ideal, 'g-', label='Traiettoria Ideale (Ruckig)', linewidth=2)
    # Disegniamo i pallini ('ko' sta per black dots) senza unire le linee
    axs[0].plot(t_raw, dist_raw, 'ko', label='Campioni Reali (Logger)', markersize=5, alpha=0.7)
    axs[0].set_ylabel('Distanza [m]')
    axs[0].set_title('Confronto Posizione')
    axs[0].grid(True); axs[0].legend()

    # Subplot 2: Velocità
    axs[1].plot(t_ideal, vel_ideal, color='orange', label='Velocità Ideale', linewidth=2)
    axs[1].plot(t_vel_raw, vel_raw, 'ko', label='Velocità Calcolata', markersize=5, alpha=0.7)
    axs[1].set_ylabel('Velocità [m/s]')
    axs[1].set_title('Confronto Velocità')
    axs[1].grid(True); axs[1].legend()

    # Subplot 3: Accelerazione
    axs[2].plot(t_ideal, acc_ideal, 'r-', label='Accelerazione Ideale', linewidth=2)
    axs[2].plot(t_acc_raw, acc_raw, 'ko', label='Accelerazione Calcolata', markersize=5, alpha=0.7)
    axs[2].set_ylabel('Accelerazione [m/s²]')
    axs[2].set_xlabel('Tempo [s]')
    axs[2].set_title('Confronto Accelerazione')
    axs[2].axhline(0, color='black', linewidth=0.8, linestyle=':')
    axs[2].grid(True); axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()