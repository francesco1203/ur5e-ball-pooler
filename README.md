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

## Usage

On LinuxUbuntu, enter the folder 'ws_ur5e_ballpool' and to build use cmd:

- colcon build --symlink-install

Give executing permission to the bash script:

- chmod +x start_robot.sh

Then, run it:

- ./start_robot.sh


### Note about using MuJoCo!!
You should modify by hand the config file ./ws_ur5e_ballpool/src/moveit_config/config/left_arm_ur5e.ros2_control.xacro, hardware section, decommenting the MuJoCo plugin (TODO: automatic upload)

Then you should upload the positions of the obstacles all by hand, modifying directly mujoco_bridge/complete_scene.xml (TODO: automatic upload)

---

### Structure

- meshes
- ws_ur5_ball_pooler: ros2 workspace

---

### Esame / Exam

(IT) : progetto per l'esame di ROBOTICA - facoltà magistrale di Ingegneria Informatica, ramo Automazione

(EN) : project for exam ROBOTICS - Master's degree in Computer Engineering, Automation branch