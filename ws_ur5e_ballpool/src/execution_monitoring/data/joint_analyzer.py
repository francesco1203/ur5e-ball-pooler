import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def main():
    # Se passi il file da riga di comando usa quello, altrimenti usa come default quello del tiro
    file_path = sys.argv[1] if len(sys.argv) > 1 else 'src/execution_monitoring/data/joint_logging/joint_log_4_shot.csv'

    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"Errore nella lettura del file {file_path}: {e}")
        return

    # Estrazione dell'asse dei tempi
    t = df['time_sec'].values
    
    # Individuiamo le colonne dei giunti (es. q0, q1, ..., q5)
    joint_cols = [col for col in df.columns if col.startswith('q')]

    if not joint_cols:
        print("Nessuna colonna giunto ('q0', 'q1', ecc.) trovata nel CSV!")
        return

    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.canvas.manager.set_window_title(f'Analisi Spazio Giunti - {file_path}')

    # Ciclo su ciascun giunto per calcolare e plottare le grandezze
    for joint in joint_cols:
        pos = df[joint].values
        vel = np.gradient(pos, t)          # Calcolo della velocità come derivata numerica della posizione
        acc = np.gradient(vel, t)          # Calcolo dell'accelerazione come derivata numerica della velocità

        axs[0].plot(t, pos, label=joint)
        axs[1].plot(t, vel, label=f'vel_{joint}')
        axs[2].plot(t, acc, label=f'acc_{joint}')

    # Configurazione Subplot 1: Posizione
    axs[0].set_ylabel('Posizione [rad]')
    axs[0].set_title('Posizione dei Giunti nel Tempo')
    axs[0].grid(True)
    axs[0].legend(loc='upper right', bbox_to_anchor=(1.12, 1.0))

    # Configurazione Subplot 2: Velocità
    axs[1].set_ylabel('Velocità [rad/s]')
    axs[1].set_title('Velocità dei Giunti nel Tempo')
    axs[1].grid(True)

    # Configurazione Subplot 3: Accelerazione
    axs[2].set_ylabel('Accelerazione [rad/s²]')
    axs[2].set_xlabel('Tempo [s]')
    axs[2].set_title('Accelerazione dei Giunti. nel Tempo')
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()