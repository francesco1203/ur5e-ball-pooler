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

- ROS2 JAZZY (framework)
- MoveIt (Ros tool for robot movements control)
- MuJoCo (Physics simulator)
- Python (for data analysis)

---

### Dipendenze / Dipendences

- ROS2Jazzy environment (source)
- ur_description (Ros2 standard package)
- moveit2 (Ros2 standard package)
- mujoco_ros2_control (Ros2 package) + mujoco + python3-yaml
- python packs: pandas, numpy, matplotlib, scipy

---

## Usage

On LinuxUbuntu, enter the folder 'ws_ur5e_ballpool' and to build use cmd:

- colcon build --symlink-install

Give executing permission to the following bash scripts:

- chmod +x start_robot.sh
- chmod +x show_plots.sh

Then, you can run the shot execution:

- ./start_robot.sh

And you can visualize the trajectory parameterization by running

- ./show_plots.sh


### Note: to use MuJoCo Vs to use RViz
If you want to use MuJoCo, you should:
- modify by hand the config file ./ws_ur5e_ballpool/src/moveit_config/config/left_arm_ur5e.ros2_control.xacro, hardware section, decommenting EXCLUSIVELY the MuJoCo plugin
- modify by hand the config file ./ws_ur5e_ballpool/src/shot_planning/config/task_params.yaml in the task_node parameters, section 'execution parameter' and set on 'true' the 'using_mujoco_simulation'parameter

Otherwise, if you want to use RViz, you should:
- modify by hand the config file ./ws_ur5e_ballpool/src/moveit_config/config/left_arm_ur5e.ros2_control.xacro, hardware section, decommenting EXCLUSIVELY the FakeHardwer plugin
- modify by hand the config file ./ws_ur5e_ballpool/src/shot_planning/config/task_params.yaml in the task_node parameters, section 'execution parameter' and set on 'false' the 'using_mujoco_simulation'parameter

Forgetting these two pre-steps, the correct execution isn't guaranteed

---

### ROS WS Structure

- ws_ur5_ball_pooler: ros2 workspace
- src/camera_perception
- src/execution_monitoring
- src/interfaces_pkg
- src/left_arm_description
- src/moveit_config
- src/scene_description
- src/shared_headers_pkg
- src/shot_planning

---

### Esame / Exam

(IT) : progetto per l'esame di ROBOTICA - facoltà magistrale di Ingegneria Informatica, ramo Automazione

(EN) : project for exam ROBOTICS - Master's degree in Computer Engineering, Automation branch