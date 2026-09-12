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

### Main Languages & Hardware Technologies

- Ur5e robotic arm
- ROS2-JAZZY (development framework)
- MoveIt (Ros tool for robot movemental control)
- MuJoCo (Physical simulator)
- Python (data analysis)
- Docker (UrSim simulation)
- IntelRealSense camera

---

### Dipendenze / Dipendences

see 'scripts/install_dependencies.sh'

---

## Usage (on LinuxUbuntu)

Execute the following bash scripts to deploy the project (NOTE: give execution permission first)

0. Install the project dependencies
- scripts/install_dependencies.sh


1. Build workspace
- scripts/build_workspace.sh


2. Setup the configs in the script and run the simulation
- scripts/start_simulated_robot.sh


3. Setup the configs in the script and run the real execution on Ur5e or UrSim simulator
- scripts/start_real_robot.sh


4. Analyze the datas from bagfiles and csv
- scripts/show_shot_parametrization.sh

---

### Note for the simulation: using MuJoCo Vs using MockHardware vs using UrSim
These following steps aren't automated yet. (TODO)

You should modify by hand the config file ./ws_ur5e_ballpool/src/moveit_config/config/ur5e.ros2_control.xacro, hardware section, decommenting EXCLUSIVELY the Hardware of your interest:
- MuJoCo plugin
- FakeHardwer plugin
- ur_robot_driver plugin with the right robot ip

By forgetting these pre-steps, the correct execution isn't guaranteed


### Esame / Exam

(IT) : progetto per l'esame di ROBOTICA - facoltà magistrale di Ingegneria Informatica, ramo Automazione e Robotica

(EN) : project for exam ROBOTICS - Master's degree in Computer Engineering, Automation and Robotics branch