## Project Name

Ur5e ball pooler

---

## State

Work in progress..

---

### Scopo del Progetto / Project Aim

(IT) : Programmare il braccio robotico UR5e per eseguire tiri di biliardo

(EN) : To program the robotic arm UR5e to execute ball-pool shots

---

### Languages & Technologies

- ROS2 JAZZY
- MoveIt
- MuJoCo

---

### Dipendenze / Dipendences

- ROS2Jazzy environment (source)
- ur_description (Ros2 standard package)
- moveit2 (Ros2 standard package)
- mujoco_ros2_control (Ros2 package) + mujoco + python >= 3.12

---

### Usage

On LinuxUbuntu, enter the folder 'ws_ur5e_ballpool' and to build use cmd:

- 1. colcon build --symlink-install
- 2. source install/setup.bash

Now, you should modify the config file ./ws_ur5e_ballpool/src/moveit_config/config/left_arm_ur5e.ros2_control.xacro, hardware section, decommenting the right plugin (only RViz def).

# No MuJoCo:

CMD1 - start MoveIt and RViz (no MuJoCo)
- 3. ros2 launch moveit_config demo.launch.py

CMD2 - build scene
- 4. ros2 run scene_description scene_builder

# Using MuJoco

CMD1 - start MoveIt, RViz and no MuJoCo
- 3. ros2 launch moveit_config mujoco_demo.launch.py

CMD2 - build scene
- 4. ros2 run scene_description scene_builder

---

### Structure

- meshes
- ws_ur5_ball_pooler: ros2 workspace

---

### Esame / Exam

(IT) : progetto per l'esame di ROBOTICA - facoltà magistrale di Ingegneria Informatica, ramo Automazione

(EN) : project for exam ROBOTICS - Master's degree in Computer Engineering, Automation branch