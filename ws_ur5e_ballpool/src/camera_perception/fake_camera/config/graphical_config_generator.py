import tkinter as tk
from tkinter import messagebox
import os
import yaml  # Preinstallato negli ambienti ROS 2 (pacchetto python3-yaml)

class BilliardSetupApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Setup Biliardo UR5e - ROS2 Jazzy")
        
        # --- Percorsi File YAML ---
        self.filepath_ws = "ws_ur5e_ballpool/src/camera_perception/fake_camera/config/fake_camera_config.yaml"
        self.filepath_local = "fake_camera_config.yaml"

        # --- Valori di Default (verranno sovrascritti se il file esiste) ---
        self.table_pos_x = -0.61
        self.table_pos_y = -0.33
        self.table_yaw = 0.0
        self.table_rialzo = 0.000
        
        self.balls = {
            "red": [0.12, 0.00],
            "white": [0.05, -0.05],
            "blue": [-0.12, 0.07],
            "yellow": [-0.10, 0.0]
        }

        # --- Caricamento configurazione salvata ---
        self.load_yaml()

        # --- Parametri di configurazione (Sincronizzati col C++) ---
        self.scale = 1000.0  # 1 metro = 1000 pixel
        
        # Dimensioni tavolo C++
        self.outer_length = 0.503  # POOL_TABLE_LENGTH
        self.outer_width = 0.304   # POOL_TABLE_WIDTH
        
        self.field_length = 0.450  # POOL_TABLE_FIELD_LENGTH
        self.field_width = 0.275   # POOL_TABLE_FIELD_WIDTH
        
        self.ball_radius_m = 0.0125 # BALL_RADIUS
        
        # Dimensioni per il canvas (pixel)
        self.ball_radius_px = self.ball_radius_m * self.scale
        self.pocket_radius_px = 18 
        
        # Dimensioni canvas e centro
        self.c_width = 800
        self.c_height = 500
        self.cx = self.c_width / 2
        self.cy = self.c_height / 2
        
        # Variabili per il drag & drop
        self.dragged_ball = None
        
        self.setup_ui()
        self.draw_table()
        self.draw_axes()
        self.draw_balls()

    def load_yaml(self):
        """Tenta di caricare il file YAML se esiste per ripristinare l'ultimo stato salvato."""
        file_to_load = None
        if os.path.exists(self.filepath_ws):
            file_to_load = self.filepath_ws
        elif os.path.exists(self.filepath_local):
            file_to_load = self.filepath_local

        if file_to_load:
            try:
                with open(file_to_load, 'r') as f:
                    data = yaml.safe_load(f)

                # Carica dati tavolo
                if 'billiard_table' in data:
                    table_data = data['billiard_table']
                    if 'pos' in table_data:
                        self.table_pos_x = table_data['pos'][0]
                        self.table_pos_y = table_data['pos'][1]
                    if 'yaw_angle_rad' in table_data:
                        self.table_yaw = table_data['yaw_angle_rad']
                    if 'rialzo_vention' in table_data:
                        self.table_rialzo = table_data['rialzo_vention']

                # Carica posizioni palline
                if 'balls' in data:
                    for ball_data in data['balls']:
                        color = ball_data.get('color')
                        pos = ball_data.get('pos')
                        # Aggiorna solo se il colore è tra quelli conosciuti
                        if color in self.balls and pos and len(pos) == 2:
                            self.balls[color] = [pos[0], pos[1]]
                            
                print(f"[*] Configurazione precedente caricata con successo da: {file_to_load}")
            except Exception as e:
                print(f"[!] Errore nel caricamento del file YAML ({e}). Verranno usati i valori di default.")

    def setup_ui(self):
        input_frame = tk.Frame(self.root, padx=10, pady=10)
        input_frame.pack(side=tk.TOP, fill=tk.X)
        
        tk.Label(input_frame, text="Pos X (World):").grid(row=0, column=0, padx=5)
        self.entry_pos_x = tk.Entry(input_frame, width=10)
        self.entry_pos_x.insert(0, str(self.table_pos_x))
        self.entry_pos_x.grid(row=0, column=1, padx=5)
        
        tk.Label(input_frame, text="Pos Y (World):").grid(row=0, column=2, padx=5)
        self.entry_pos_y = tk.Entry(input_frame, width=10)
        self.entry_pos_y.insert(0, str(self.table_pos_y))
        self.entry_pos_y.grid(row=0, column=3, padx=5)
        
        tk.Label(input_frame, text="Yaw (rad):").grid(row=0, column=4, padx=5)
        self.entry_yaw = tk.Entry(input_frame, width=10)
        self.entry_yaw.insert(0, str(self.table_yaw))
        self.entry_yaw.grid(row=0, column=5, padx=5)
        
        tk.Label(input_frame, text="Rialzo Vention:").grid(row=0, column=6, padx=5)
        self.entry_rialzo = tk.Entry(input_frame, width=10)
        self.entry_rialzo.insert(0, str(self.table_rialzo))
        self.entry_rialzo.grid(row=0, column=7, padx=5)

        self.canvas = tk.Canvas(self.root, width=self.c_width, height=self.c_height, bg="#e0e0e0")
        self.canvas.pack(pady=10)
        
        self.canvas.bind("<ButtonPress-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        
        self.info_label = tk.Label(self.root, text="Trascina le palline per posizionarle. Collisioni con sponde attivate.", fg="blue")
        self.info_label.pack()

        btn_frame = tk.Frame(self.root, pady=10)
        btn_frame.pack(side=tk.BOTTOM)
        tk.Button(btn_frame, text="Sovrascrivi YAML scena", command=self.generate_yaml, bg="green", fg="white", font=("Arial", 12, "bold")).pack()

    def ros2canvas(self, x_ros, y_ros):
        # X verso sinistra, Y verso il BASSO
        x_c = self.cx - (x_ros * self.scale)
        y_c = self.cy + (y_ros * self.scale)
        return x_c, y_c

    def canvas2ros(self, x_c, y_c):
        x_ros = (self.cx - x_c) / self.scale
        y_ros = (y_c - self.cy) / self.scale
        return x_ros, y_ros

    def draw_table(self):
        # 1. Tavolo esterno (Sponde in legno)
        half_out_l = self.outer_length / 2
        half_out_w = self.outer_width / 2
        tl_out_x, tl_out_y = self.ros2canvas(half_out_l, -half_out_w)
        br_out_x, br_out_y = self.ros2canvas(-half_out_l, half_out_w)
        self.canvas.create_rectangle(tl_out_x, tl_out_y, br_out_x, br_out_y, fill="#6b4423", outline="#3d2612", width=3)

        # 2. Area di gioco interna (Panno verde)
        half_in_l = self.field_length / 2
        half_in_w = self.field_width / 2
        tl_in_x, tl_in_y = self.ros2canvas(half_in_l, -half_in_w)
        br_in_x, br_in_y = self.ros2canvas(-half_in_l, half_in_w)
        self.canvas.create_rectangle(tl_in_x, tl_in_y, br_in_x, br_in_y, fill="#2c6b3f", outline="black", width=2)
        
        # 3. Buche (calcolate sugli angoli e centri dell'area interna)
        pockets_ros = [
            (half_in_l, half_in_w),    
            (0.0, half_in_w),       
            (-half_in_l, half_in_w),   
            (half_in_l, -half_in_w),   
            (0.0, -half_in_w),      
            (-half_in_l, -half_in_w)   
        ]
        pr = self.pocket_radius_px
        for px_ros, py_ros in pockets_ros:
            cx, cy = self.ros2canvas(px_ros, py_ros)
            self.canvas.create_oval(cx - pr, cy - pr, cx + pr, cy + pr, fill="#1a1a1a", outline="#4a4a4a")
            
        self.canvas.create_oval(self.cx-4, self.cy-4, self.cx+4, self.cy+4, fill="white")

    def draw_axes(self):
        self.canvas.create_line(self.cx, self.cy, self.cx - 80, self.cy, arrow=tk.LAST, fill="red", width=2)
        self.canvas.create_text(self.cx - 95, self.cy, text="X", fill="red", font=("Arial", 10, "bold"))
        self.canvas.create_line(self.cx, self.cy, self.cx, self.cy + 80, arrow=tk.LAST, fill="cyan", width=2)
        self.canvas.create_text(self.cx, self.cy + 95, text="Y", fill="cyan", font=("Arial", 10, "bold"))

    def draw_balls(self):
        self.ball_items = {}
        for color, (x, y) in self.balls.items():
            cx, cy = self.ros2canvas(x, y)
            r = self.ball_radius_px
            outline_col = "gray" if color == "white" else "black"
            item_id = self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill=color, outline=outline_col, width=1, tags=(color, "ball"))
            self.ball_items[item_id] = color

    def on_click(self, event):
        items = self.canvas.find_withtag("current")
        if items and items[0] in self.ball_items:
            self.dragged_ball = items[0]

    def on_drag(self, event):
        if self.dragged_ball:
            color = self.ball_items[self.dragged_ball]
            r = self.ball_radius_px
            
            # Converte il click del mouse in coordinate ROS
            raw_ros_x, raw_ros_y = self.canvas2ros(event.x, event.y)
            
            # --- LIMITE FISICO DEL TAVOLO ---
            max_x = (self.field_length / 2) - self.ball_radius_m
            max_y = (self.field_width / 2) - self.ball_radius_m
            
            # Blocca le coordinate ROS (clamp) entro il panno verde
            ros_x = max(-max_x, min(max_x, raw_ros_x))
            ros_y = max(-max_y, min(max_y, raw_ros_y))
            
            # Riconverte le coordinate bloccate in pixel per spostare la grafica
            cx, cy = self.ros2canvas(ros_x, ros_y)
            self.canvas.coords(self.dragged_ball, cx - r, cy - r, cx + r, cy + r)
            
            self.balls[color] = [ros_x, ros_y]
            self.info_label.config(text=f"Pallina {color.upper()}: x={ros_x:.3f}, y={ros_y:.3f}")

    def on_release(self, event):
        self.dragged_ball = None

    def generate_yaml(self):
        try:
            pos_x = float(self.entry_pos_x.get())
            pos_y = float(self.entry_pos_y.get())
            yaw = float(self.entry_yaw.get())
            rialzo = float(self.entry_rialzo.get())
        except ValueError:
            messagebox.showerror("Errore", "Per favore, inserisci valori numerici validi nei campi del tavolo.")
            return

        yaml_content = f"""billiard_table:
  pos: [{pos_x}, {pos_y}]                   # Posizionamento sul banco rispetto al world
  yaw_angle_rad: {yaw}                    # Orientamento sul banco rispetto al world  
  rialzo_vention: {rialzo:.3f}                 # Rialzo del tavolo rispetto al banco
#per ora usiamo solo palline piene, possibile futura implementazione -> type: "solid" o "striped"

balls:"""
        
        first_ball = True
        for color, (x, y) in self.balls.items():
            comment = '                     # Posizione rispetto al centro del tavolo' if first_ball else ''
            yaml_content += f'\n  - color: "{color}"\n    pos: [{x:.3f}, {y:.3f}]{comment}'
            first_ball = False

        try:
            with open(self.filepath_ws, 'w') as f:
                f.write(yaml_content)
            messagebox.showinfo("Successo", f"File '{self.filepath_ws}' generato correttamente!\n\nLe posizioni sono state salvate.")
        except FileNotFoundError:
            with open(self.filepath_local, 'w') as f:
                f.write(yaml_content)
            messagebox.showwarning("Attenzione", f"Directory non trovata.\nIl file è stato salvato nella cartella corrente come:\n{self.filepath_local}")

if __name__ == "__main__":
    root = tk.Tk()
    app = BilliardSetupApp(root)
    root.mainloop()