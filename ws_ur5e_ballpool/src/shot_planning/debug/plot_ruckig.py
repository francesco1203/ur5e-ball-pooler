import sys
import pandas as pd
import matplotlib.pyplot as plt

def main():
    # Se passi il file da riga di comando usa quello, altrimenti usa come default quello appena creato dal C++
    file_path = sys.argv[1] if len(sys.argv) > 1 else 'src/shot_planning/debug/ruckig_logging/ruckig_trajectory_log.csv'

    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"Errore nella lettura del file {file_path}: {e}")
        return

    # Estrazione delle coordinate temporali e spaziali direttamente calcolate da Ruckig
    # Intestazione generata nel C++: "Time_s,Position_m,Velocity_ms,Acceleration_ms2"
    t = df['Time_s'].values
    pos = df['Position_m'].values
    vel = df['Velocity_ms'].values
    acc = df['Acceleration_ms2'].values

    # Plotting
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title(f'Analisi Ruckig Ideale - {file_path}')

    # Subplot 1: Distanza
    axs[0].plot(t, pos, 'g-', label='Distanza percorsa ideale [m]')
    axs[0].set_ylabel('Distanza [m]')
    axs[0].set_title('Profilo di Posizione Ideale (Ruckig)')
    axs[0].grid(True)
    axs[0].legend()

    # Subplot 2: Velocità
    axs[1].plot(t, vel, color='orange', label='Velocità ideale [m/s]')
    axs[1].set_ylabel('Velocità [m/s]')
    axs[1].set_title('Profilo di Velocità Ideale (Ruckig)')
    axs[1].grid(True)
    axs[1].legend()

    # Subplot 3: Accelerazione
    axs[2].plot(t, acc, color='red', label='Accelerazione ideale [m/s²]')
    axs[2].set_ylabel('Accelerazione [m/s²]')
    axs[2].set_xlabel('Tempo [s]')
    axs[2].set_title('Profilo di Accelerazione Ideale (Ruckig)')
    axs[2].axhline(0, color='black', linewidth=0.8, linestyle=':')
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
