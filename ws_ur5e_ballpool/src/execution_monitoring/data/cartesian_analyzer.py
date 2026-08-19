import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def main():
    # 1. ESTRAZIONE FILE DA DATI

    # Se passi il file da riga di comando usa quello, altrimenti usa come default quello del tiro
    file_path = sys.argv[1] if len(sys.argv) > 1 else 'src/execution_monitoring/data/cartesian_logging/cartesian_log_4_shot.csv'

    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"Errore nella lettura del file {file_path}: {e}")
        return

    # Estrazione delle coordinate temporali e spaziali
    t = df['time_sec'].values
    x = df['x'].values
    y = df['y'].values
    z = df['z'].values


    # 2. CALCOLO DI DISTANZA, VELOCITÀ E ACCELERAZIONE CARTESIANE

    # filtering variables (QUANDO FILTRO, LE TRAIETTORIE SONO IRRICONOSCIBILI, MEGLIO NON FARLO)
    # filtering_distance = 'false'
    # filtering_velocity = 'false'
    # filtering_acceleration = 'false'


    # Distanza percorsa dall'inizio
    dist_from_start = np.sqrt((x - x[0])**2 + (y - y[0])**2 + (z - z[0])**2)

    # Se richiesto, applica un filtro per smussare i gradini 
    # if filtering_distance == 'true':
    #     dist_from_start = savgol_filter(dist_from_start, window_length=15, polyorder=3)

    
    # Derivate numeriche per Velocità
    vel_cart = np.gradient(dist_from_start, t)

    # Se richiesto, applica un filtro per smussare i gradini 
    # if filtering_velocity == 'true':
    #     vel_cart = savgol_filter(vel_cart, window_length=15, polyorder=3)


    # Derivate numeriche per Accelerazione
    acc_cart = np.gradient(vel_cart, t)

    # Se richiesto, applica un filtro per smussare i gradini 
    # if filtering_acceleration == 'true':
    #     acc_cart = savgol_filter(acc_cart, window_length=15, polyorder=3)



    # 3. PLOTTING DEI RISULTATI
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.canvas.manager.set_window_title(f'Analisi Cartesiana - {file_path}')

    # Subplot 1: Distanza
    axs[0].plot(t, dist_from_start, 'g-', label='Distanza percorsa [m]')
    axs[0].set_ylabel('Distanza [m]')
    axs[0].set_title('Profilo di Posizione Cartesiana')
    axs[0].grid(True)
    axs[0].legend()

    # Subplot 2: Velocità
    axs[1].plot(t, vel_cart, color='orange', label='Velocità cartesiana [m/s]')
    axs[1].set_ylabel('Velocità [m/s]')
    axs[1].set_title('Profilo di Velocità Cartesiana')
    axs[1].grid(True)
    axs[1].legend()

    # Subplot 3: Accelerazione
    axs[2].plot(t, acc_cart, color='red', label='Accelerazione cartesiana [m/s²]')
    axs[2].set_ylabel('Accelerazione [m/s²]')
    axs[2].set_xlabel('Tempo [s]')
    axs[2].set_title('Profilo di Accelerazione Cartesiana')
    axs[2].axhline(0, color='black', linewidth=0.8, linestyle=':')
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()