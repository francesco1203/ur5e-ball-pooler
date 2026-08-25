import yaml
import math
import argparse
from pathlib import Path

# --- SETUP ARGOMENTI DA RIGA DI COMANDO ---
parser = argparse.ArgumentParser(description="Genera la scena MuJoCo a partire da un file YAML.")

# Calcoliamo il percorso di default per mantenere il comportamento originale
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = SCRIPT_DIR.parent / "config" / "fake_camera_config.yaml"

# Aggiungiamo l'argomento (opzionale)
parser.add_argument(
    '--yaml_path',
    type=str,
    default=str(DEFAULT_CONFIG_PATH),
    help="Percorso assoluto o relativo al file YAML di configurazione."
)

# Parsa gli argomenti passati da terminale
args = parser.parse_args()


# --- GESTIONE DEI PERCORSI CON PATHLIB ---

# Definisco i percorsi relativi partendo da SCRIPT_DIR
PATH_TEMPLATE = SCRIPT_DIR / "scene_template.xml"

# Uso il percorso passato da riga di comando (o il default se non è stato passato nulla)
PATH_CONFIG = Path(args.yaml_path)

# Per l'output: risali di 3 livelli (parent[2]) fino a 'src', poi entra in 'moveit_config/config/mujoco_bridge'
PATH_OUTPUT = SCRIPT_DIR.parents[2] / "moveit_config" / "config" / "mujoco_bridge" / "complete_scene.xml"


# --- ESECUZIONE DELLA LOGICA ---

# 0. Assicurati che la cartella di destinazione esista (la crea se non c'è)
PATH_OUTPUT.parent.mkdir(parents=True, exist_ok=True)


# 1. Leggi i file (testo XML puro e YAML)
with open(PATH_TEMPLATE, "r") as f:
    xml_template = f.read()

# Apri il file YAML usando PATH_CONFIG
with open(PATH_CONFIG, "r") as f:
    config = yaml.safe_load(f)


# 2. Estrai dati tavolo
tavolo_x, tavolo_y = config['billiard_table']['pos']
tavolo_yaw = config['billiard_table']['yaw_angle_rad']
tavolo_z = 0.0
z_palline = 0.0725

# Prepariamo un dizionario con i valori finali da iniettare
valori_da_iniettare = {
    "tavolo_pos": f"{tavolo_x} {tavolo_y} {tavolo_z}",
    "tavolo_yaw": f"{tavolo_yaw}",
    # Valori di default nel caso mancassero nel YAML (li mettiamo "sotto terra" o fuori vista)
    "white_pos": "0 0 -5",
    "red_pos": "0 0 -5",
    "blue_pos": "0 0 -5",
    "yellow_pos": "0 0 -5",
}


# 3. Calcola posizioni assolute delle palline
for pallina in config.get('balls', []):
    colore = pallina['color']
    rel_x, rel_y = pallina['pos']       #posizione relativa al tavolo -> devo trasformarla in posizione assoluta

    # Le posizioni sono intese rispetto alla terna di centro biliardo, che rispetto a world, ha x'=-x e y'=y
    # Dunque, devo cambiare il segno alle cordinate x e y
    rel_x = -rel_x
    rel_y = -rel_y

    # Rotazione (se il tavolo è girato)
    rot_x = rel_x * math.cos(tavolo_yaw) - rel_y * math.sin(tavolo_yaw)
    rot_y = rel_x * math.sin(tavolo_yaw) + rel_y * math.cos(tavolo_yaw)

    abs_x = tavolo_x + rot_x
    abs_y = tavolo_y + rot_y

    # Aggiorna il dizionario con la posizione calcolata
    chiave = f"{colore}_pos" # es. diventa "white_pos" o "red_pos"
    valori_da_iniettare[chiave] = f"{abs_x:.5f} {abs_y:.5f} {z_palline}"


# 4. Magia del format: sostituisce i {...} nell'XML con i valori!
xml_finale = xml_template.format(**valori_da_iniettare)


# 5. Salva il risultato in un file 
with open(PATH_OUTPUT, "w") as f:
    f.write(xml_finale)