import tkinter as tk
from tkinter import messagebox
import os
import yaml
import random

class DemoMenuApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Menu Biliardo UR5e")
        self.root.geometry("350x200") 
        
        tk.Label(self.root, text="UR5e Billiard Demo", font=("Arial", 16, "bold")).pack(pady=15)

        tk.Button(self.root, text="Play Random", command=self.play_random, bg="orange", width=20, font=("Arial", 12)).pack(pady=5)
        tk.Button(self.root, text="Play Place Balls", command=self.play_place_balls, bg="lightblue", width=20, font=("Arial", 12)).pack(pady=5)
        tk.Button(self.root, text="Exit", command=self.root.quit, bg="indianred", fg="white", width=20, font=("Arial", 12)).pack(pady=5)

    def play_random(self):
        BilliardSetupApp(None, mode="random", main_menu_ref=self, headless=True)

    def play_place_balls(self):
        self.open_setup(mode="manual")

    def open_setup(self, mode):
        self.root.withdraw()
        setup_window = tk.Toplevel(self.root)
        setup_window.protocol("WM_DELETE_WINDOW", self.on_setup_close)
        BilliardSetupApp(setup_window, mode, self, headless=False)

    def on_setup_close(self):
        self.root.deiconify()


class BilliardSetupApp:
    # Configurazioni predefinite esatte per le palline
    PRESET_CONFIGURATIONS = [
         {"white": [0.139, 0.014], "red": [0.196, -0.066]},
         {"white": [-0.10, -0.05],   "red": [0.05, 0.05]},
        {"white": [-0.121, -0.013],    "red": [-0.096, -0.06]},
         {"white": [-0.004, 0.042],   "red": [0.133, 0.071]},
         {"white": [-0.009, -0.049],    "red": [-0.127, -0.09]},
         {"white": [-0.125, -0.015],    "red": [-0.155, 0.077]},
         {"white": [0.082, -0.010],   "red": [0.018, -0.069]},
         {"white": [-0.109, -0.048],    "red": [0.081, -0.106]},
         {"white": [0.076, -0.023],    "red": [0.05, 0.07]},
        {"white": [-0.036, -0.012],   "red": [-0.135, -0.055]},
    ]

    def __init__(self, root, mode, main_menu_ref, headless=False):
        self.root = root
        self.mode = mode
        self.main_menu_ref = main_menu_ref
        self.headless = headless
        
        self.filepath_ws = "ws_ur5e_ballpool/src/camera_perception/fake_camera/config/fake_camera_config.yaml"
        self.filepath_local = "fake_camera_config.yaml"

        self.table_pos_x = -0.61
        self.table_pos_y = -0.33
        self.table_yaw = 0.0
        self.table_rialzo = 0.000
        
        self.balls = {}

        self.scale = 1000.0  
        self.outer_length = 0.503  
        self.outer_width = 0.304   
        self.field_length = 0.450  
        self.field_width = 0.275   
        self.ball_radius_m = 0.0125 
        
        self.ball_radius_px = self.ball_radius_m * self.scale
        self.pocket_radius_px = 18 
        self.c_width, self.c_height = 800, 500
        self.cx, self.cy = self.c_width / 2, self.c_height / 2
        self.dragged_ball = None

        self.allowed_colors = ["white", "red"]

        if self.mode == "manual":
            default_positions = {
                "white": [0.05, -0.05],
                "red": [0.12, 0.00]
            }
            self.balls = {c: default_positions[c] for c in self.allowed_colors}
            self.load_yaml(load_balls=True)
        elif self.mode == "random":
            self.load_yaml(load_balls=False)
            self.select_preset_balls()
        
        if self.headless:
            self.generate_yaml(show_message=False)
        else:
            self.root.title(f"Setup Biliardo UR5e - Modalità: {mode.upper()}")
            self.setup_ui()
            self.draw_table()
            self.draw_axes()
            self.draw_balls()

    def select_preset_balls(self):
        """Seleziona casualmente una delle configurazioni predefinite."""
        self.balls.clear()
        chosen_preset = random.choice(self.PRESET_CONFIGURATIONS)
        
        for color in self.allowed_colors:
            if color in chosen_preset:
                self.balls[color] = list(chosen_preset[color])

    def load_yaml(self, load_balls=True):
        file_to_load = self.filepath_ws if os.path.exists(self.filepath_ws) else self.filepath_local if os.path.exists(self.filepath_local) else None
        if file_to_load:
            try:
                with open(file_to_load, 'r') as f: 
                    data = yaml.safe_load(f)
                if 'billiard_table' in data:
                    table_data = data['billiard_table']
                    self.table_pos_x = table_data.get('pos', [self.table_pos_x])[0]
                    self.table_pos_y = table_data.get('pos', [0, self.table_pos_y])[1]
                    self.table_yaw = table_data.get('yaw_angle_rad', self.table_yaw)
                    self.table_rialzo = table_data.get('rialzo_vention', self.table_rialzo)

                if load_balls and 'balls' in data:
                    loaded_balls = {}
                    for b in data['balls']:
                        c, p = b.get('color'), b.get('pos')
                        if c in self.allowed_colors and p and len(p) == 2:
                            loaded_balls[c] = [p[0], p[1]]
                    if loaded_balls:
                        self.balls = loaded_balls
            except Exception: 
                pass

    def setup_ui(self):
        input_frame = tk.Frame(self.root, padx=10, pady=10)
        input_frame.pack(side=tk.TOP, fill=tk.X)
        
        tk.Label(input_frame, text="Pos X:").grid(row=0, column=0, padx=5)
        self.entry_pos_x = tk.Entry(input_frame, width=8)
        self.entry_pos_x.insert(0, str(self.table_pos_x))
        self.entry_pos_x.grid(row=0, column=1, padx=5)
        
        tk.Label(input_frame, text="Pos Y:").grid(row=0, column=2, padx=5)
        self.entry_pos_y = tk.Entry(input_frame, width=8)
        self.entry_pos_y.insert(0, str(self.table_pos_y))
        self.entry_pos_y.grid(row=0, column=3, padx=5)
        
        tk.Label(input_frame, text="Yaw(rad):").grid(row=0, column=4, padx=5)
        self.entry_yaw = tk.Entry(input_frame, width=8)
        self.entry_yaw.insert(0, str(self.table_yaw))
        self.entry_yaw.grid(row=0, column=5, padx=5)
        
        tk.Label(input_frame, text="Rialzo:").grid(row=0, column=6, padx=5)
        self.entry_rialzo = tk.Entry(input_frame, width=8)
        self.entry_rialzo.insert(0, str(self.table_rialzo))
        self.entry_rialzo.grid(row=0, column=7, padx=5)

        self.canvas = tk.Canvas(self.root, width=self.c_width, height=self.c_height, bg="#e0e0e0")
        self.canvas.pack(pady=10)
        
        self.canvas.bind("<ButtonPress-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)
        
        msg = f"Trascina le {len(self.balls)} palline per posizionarle." if self.mode == "manual" else "Configurazione predefinita caricata."
        self.info_label = tk.Label(self.root, text=msg, fg="blue")
        self.info_label.pack()

        btn_frame = tk.Frame(self.root, pady=10)
        btn_frame.pack(side=tk.BOTTOM)
        
        tk.Button(btn_frame, text="Salva e Chiudi", command=self.generate_yaml, bg="green", fg="white", font=("Arial", 12, "bold")).pack(side=tk.LEFT, padx=10)
        tk.Button(btn_frame, text="Annulla", command=self.close_without_saving, bg="gray", fg="white", font=("Arial", 12)).pack(side=tk.LEFT, padx=10)

    def close_without_saving(self):
        if self.root:
            self.root.destroy()
        if self.main_menu_ref and self.main_menu_ref.root:
            self.main_menu_ref.root.deiconify()

    def ros2canvas(self, x_ros, y_ros):
        x_c = self.cx - (x_ros * self.scale)
        y_c = self.cy + (y_ros * self.scale)
        return x_c, y_c

    def canvas2ros(self, x_c, y_c):
        x_ros = (self.cx - x_c) / self.scale
        y_ros = (y_c - self.cy) / self.scale
        return x_ros, y_ros

    def draw_table(self):
        half_out_l = self.outer_length / 2
        half_out_w = self.outer_width / 2
        tl_out_x, tl_out_y = self.ros2canvas(half_out_l, -half_out_w)
        br_out_x, br_out_y = self.ros2canvas(-half_out_l, half_out_w)
        self.canvas.create_rectangle(tl_out_x, tl_out_y, br_out_x, br_out_y, fill="#6b4423", outline="#3d2612", width=3)

        half_in_l = self.field_length / 2
        half_in_w = self.field_width / 2
        tl_in_x, tl_in_y = self.ros2canvas(half_in_l, -half_in_w)
        br_in_x, br_in_y = self.ros2canvas(-half_in_l, half_in_w)
        self.canvas.create_rectangle(tl_in_x, tl_in_y, br_in_x, br_in_y, fill="#2c6b3f", outline="black", width=2)
        
        pockets_ros = [
            (half_in_l, half_in_w), (0.0, half_in_w), (-half_in_l, half_in_w),   
            (half_in_l, -half_in_w), (0.0, -half_in_w), (-half_in_l, -half_in_w)   
        ]
        for px_ros, py_ros in pockets_ros:
            cx, cy = self.ros2canvas(px_ros, py_ros)
            self.canvas.create_oval(cx - self.pocket_radius_px, cy - self.pocket_radius_px, cx + self.pocket_radius_px, cy + self.pocket_radius_px, fill="#1a1a1a", outline="#4a4a4a")
            
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
            raw_ros_x, raw_ros_y = self.canvas2ros(event.x, event.y)
            max_x = (self.field_length / 2) - self.ball_radius_m
            max_y = (self.field_width / 2) - self.ball_radius_m
            
            ros_x = max(-max_x, min(max_x, raw_ros_x))
            ros_y = max(-max_y, min(max_y, raw_ros_y))
            
            cx, cy = self.ros2canvas(ros_x, ros_y)
            self.canvas.coords(self.dragged_ball, cx - r, cy - r, cx + r, cy + r)
            self.balls[color] = [ros_x, ros_y]
            self.info_label.config(text=f"Pallina {color.upper()}: x={ros_x:.3f}, y={ros_y:.3f}")

    def on_release(self, event):
        self.dragged_ball = None

    def generate_yaml(self, show_message=True):
        if not self.headless:
            try:
                self.table_pos_x = float(self.entry_pos_x.get())
                self.table_pos_y = float(self.entry_pos_y.get())
                self.table_yaw = float(self.entry_yaw.get())
                self.table_rialzo = float(self.entry_rialzo.get())
            except ValueError:
                messagebox.showerror("Errore", "Inserisci valori numerici validi nei campi del tavolo.")
                return

        yaml_content = f"""billiard_table:
  pos: [{self.table_pos_x}, {self.table_pos_y}]
  yaw_angle_rad: {self.table_yaw}
  rialzo_vention: {self.table_rialzo:.3f}

balls:"""
        
        first_ball = True
        for color, (x, y) in self.balls.items():
            comment = '  # Posizione rispetto al centro del tavolo' if first_ball else ''
            # Corretta la formattazione YAML rimuovendo gli accapo errati:
            yaml_content += f'\n  - color: "{color}"\n    pos: [{x:.3f}, {y:.3f}]{comment}'
            first_ball = False

        try:
            with open(self.filepath_ws, 'w') as f: 
                f.write(yaml_content)
            if show_message: 
                messagebox.showinfo("Successo", f"File '{self.filepath_ws}' generato!")
        except FileNotFoundError:
            with open(self.filepath_local, 'w') as f: 
                f.write(yaml_content)
            if show_message: 
                messagebox.showwarning("Attenzione", f"Directory non trovata.\nSalvato: {self.filepath_local}")
        
        if self.root: 
            self.root.destroy()
        if self.main_menu_ref and self.main_menu_ref.root:
            self.main_menu_ref.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = DemoMenuApp(root)
    root.mainloop()