import csv
import matplotlib.pyplot as plt

iterations, err_p, err_o = [], [], []
q = [[] for _ in range(6)]

# Leggi il file CSV prodotto da ROS 2
with open('clik_data.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Salta l'intestazione
    for row in reader:
        iterations.append(float(row[0]))
        for i in range(6):
            q[i].append(float(row[1 + i]))
        err_p.append(float(row[7]))
        err_o.append(float(row[8]))

# Grafico
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

for i in range(6):
    ax1.plot(iterations, q[i], label=f'Giunto q_{i}')
ax1.set_ylabel('Posizione Giunti [rad]')
ax1.set_title('Evoluzione delle 6 Variabili di Giunto')
ax1.grid(True)
ax1.legend(ncol=2)

ax2.plot(iterations, err_p, label='Errore Posizione [m]', color='red')
ax2.plot(iterations, err_o, label='Errore Orientamento [rad]', color='blue', linestyle='--')
ax2.set_xlabel('Iterazione')
ax2.set_ylabel('Errore')
ax2.set_yscale('log')
ax2.set_title('Evoluzione dell\'Errore CLIK')
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()
