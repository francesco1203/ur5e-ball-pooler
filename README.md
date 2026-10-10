## Project Name

Ur5e ball pooler

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
- scripts/install_dependencies.sh  <br> <br>
1. Build workspace
- scripts/build_workspace.sh


Then, setup the config parameters in the script and in the config files* and run:

<br> <br>
2. the simulation on RvizOnly (Mock) or MuJoCo simulator
- scripts/start_simulated_robot.sh

Or you can also easily run the made-up MuJoCo demo
- scripts/run_demo_oneshot.sh

<br> <br>
3. the simulation on simulator UrSim (TeachPendant simulator)
- scripts/start_real_robot.sh

<br> <br>
4. the real execution on Ur5e arm
- scripts/start_real_robot.sh


If you just want to test the perception algorithm and the environemnt setup, you can just run this script setting up the flag "execute_shot" to false.

<br> <br>
5. If you want to test real camera calibration using rod tip, then run:
- scripts/test_tip_positioning_real_robot.sh

<br> <br>
6. If you want to analyze the datas from bagfiles and csv
- scripts/analyze_shot_parametrization.sh COLOR PREFIX

Then check the folder data/results

<br> <br>
7.  If you want to play the videos taken by the RealSense, you can set the path to your bagfile in the following script and run 
- scripts/bag_play_camera.sh

<br> <br>
Config files* (in ws)
- shot_execution/game_engine/config/*
- shot_execution/shot_planning/config/*

---

### Esame / Exam

(IT) : progetto per l'esame di ROBOTICA - facoltà magistrale di Ingegneria Informatica, ramo Automazione e Robotica

(EN) : project for exam ROBOTICS - Master's degree in Computer Engineering, Automation and Robotics branch