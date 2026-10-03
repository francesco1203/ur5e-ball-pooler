"""
plot_ruckig.py

Visualizza il csv generato da Ruckig con i dati ideali di posizione, velocità e accelerazione (riferimento durante il tiro).
Supporta il salvataggio automatico in una sottocartella in data/results 
se viene passato il flag --save <nome_cartella>.
"""

import sys
import os
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt

def main():
    print("Plot Ruckig - Visualizza la parametrizzazione ideale di posizione, velocità e accelerazione del tiro, generati da Ruckig in un file CSV.")

    # --- Lettura Argomenti, Flag e Sottocartella ---
    args = sys.argv[1:]
    if len(args) < 1:
        print("Uso: python3 plot_ruckig.py <file_csv> [--save <sottocartella>]")
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
        print("Errore: Manca il percorso al file CSV.")
        sys.exit(1)
        
    file_path = args[0]

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
        save_file = results_dir / "ruckig.png"
        
        fig.savefig(save_file, dpi=300)
        print(f"✅ Grafico salvato in: {save_file}")
        
        plt.close(fig) 
    else:
        plt.show()

if __name__ == '__main__':
    main()