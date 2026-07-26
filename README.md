## Nome del Progetto / Project Name

Ur5e ball pooler

---

## Stato / State

Work in progress..

---

### Scopo del Progetto / Project Aim

(IT) : Programmare il braccio robotico UR5e per eseguire tiri di biliardo

(EN) : To program the robotics arm UR5e to execute ball-pool shots

---

### Linguaggi e tecnologie / Languages & Technologies

- ROS2 JAZZY
- (... MuJoCo)
- ...

---

### Dipendenze / Dipendences

- ur_description (Ros2 standard package)

---

### Usage

On LinuxUbuntu, enter the folder 'ws_ur5e_ballpool' and use cmd:

- 1. colcon build --symlink-install

CMD1 - start MoveIt and RViz
- 2. source install/setup.bash
- 3. ros2 launch moveit_config demo.launch.py

CMD2 - build scene
- 4. source install/setup.bash
- 5. ros2 run scene_description scene_builder

### IMPORTANT

Satisfy dependecies first...

---

### Structure

- meshes
- ws_ur5_ball_pooler
- ‎ /src/left_arm_description: URDF and XACRO for arm
- ‎ /src/scene_description
- ‎ /src/moveit_config

---

### Esame / Exam

(IT) : progetto per l'esame di ROBOTICA - facoltà magistrale di Ingegneria Informatica, ramo Automazione

(EN) : project for exam ROBOTICS - Master's degree in Computer Engineering, Automation branch