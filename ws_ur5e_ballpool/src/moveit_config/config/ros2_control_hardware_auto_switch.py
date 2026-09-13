#file python per aggiornare automaticamente il plugin di hardware in ur5e.ros2_control.xacro
#non generato da moveit_setup_assistant, ma creato da me per facilitare la gestione dei plugin di hardware in base alla modalità di simulazione

import argparse
import re
import sys

def get_hardware_section(mode, robot_ip, reverse_ip):
    """Genera la sezione hardware con il plugin corretto decommentato e gli IP aggiornati."""
    
    hw = ["<hardware>\n"]
    
    # 1. MUJOCO
    hw.append("                <!-- PLUGIN PER MUJOCO -->\n")
    if mode == 'mujoco':
        hw.append("                <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>\n")
        hw.append("                <param name=\"mujoco_model\">$(find moveit_config)/config/mujoco_bridge/complete_scene.xml</param>\n")
    else:
        hw.append("                <!-- <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>\n")
        hw.append("                <param name=\"mujoco_model\">$(find moveit_config)/config/mujoco_bridge/complete_scene.xml</param> -->\n")

    hw.append("\n")

    # 2. MOCK / RVIZ
    hw.append("                <!-- PLUGIN PER SOLO RVIZ SIMULATO -->\n")
    if mode == 'mock':
        hw.append("                <plugin>mock_components/GenericSystem</plugin>\n")
    else:
        hw.append("                <!-- <plugin>mock_components/GenericSystem</plugin> -->\n")

    hw.append("\n")

    # 3. REAL / URSIM
    hw.append("                <!-- PLUGIN PER HARDWARE REALE / UrSim -->\n")
    if mode == 'driver':
        hw.append("                <plugin>ur_robot_driver/URPositionHardwareInterface</plugin>\n")
        hw.append(f"                <param name=\"robot_ip\">{robot_ip}</param>\n")
        hw.append("                <param name=\"script_filename\">$(find ur_robot_driver)/resources/ros_control.urscript</param>\n")
        hw.append("                <param name=\"headless_mode\">true</param>\n")
        hw.append(f"                <param name=\"reverse_ip\">{reverse_ip}</param>\n")
        hw.append("                <param name=\"reverse_port\">50001</param>\n")
        hw.append("                <param name=\"script_sender_port\">50002</param>\n")
        hw.append("                <param name=\"script_command_port\">50004</param>\n")
        hw.append("                <param name=\"trajectory_port\">50003</param>\n")
    else:
        hw.append("                <!-- <plugin>ur_robot_driver/URPositionHardwareInterface</plugin>\n")
        hw.append(f"                <param name=\"robot_ip\">{robot_ip}</param>\n")
        hw.append("                <param name=\"script_filename\">$(find ur_robot_driver)/resources/ros_control.urscript</param>\n")
        hw.append("                <param name=\"headless_mode\">true</param>\n")
        hw.append(f"                <param name=\"reverse_ip\">{reverse_ip}</param>\n")
        hw.append("                <param name=\"reverse_port\">50001</param>\n")
        hw.append("                <param name=\"script_sender_port\">50002</param>\n")
        hw.append("                <param name=\"script_command_port\">50004</param>\n")
        hw.append("                <param name=\"trajectory_port\">50003</param> -->\n")

    hw.append("            </hardware>")
    return "".join(hw)

def main():
    parser = argparse.ArgumentParser(description="Modifica i plugin in ur5e.ros2_control.xacro")
    parser.add_argument("mode", choices=["mujoco", "mock", "driver"], help="Simulatore o hardware da attivare")
    parser.add_argument("--robot_ip", default="192.168.56.101", help="Indirizzo IP del robot (usato in modalità driver)")
    parser.add_argument("--reverse_ip", default="192.168.56.1", help="Reverse IP (usato in modalità driver)")
    
    args = parser.parse_args()


    # Legge il contenuto del file ur5e.ros2_control.xacro
    file_path = "ws_ur5e_ballpool/src/moveit_config/config/ur5e.ros2_control.xacro"
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Errore: File '{file_path}' non trovato.")
        sys.exit(1)

    # Genera il nuovo blocco
    new_hardware_section = get_hardware_section(args.mode, args.robot_ip, args.reverse_ip)

    # regex per trovare il tag <hardware> ... </hardware>
    pattern = r'<hardware>.*?</hardware>'
    new_content, count = re.subn(pattern, new_hardware_section, content, flags=re.DOTALL)

    if count == 0:
        print("Errore: Tag <hardware> non trovato nel file.")
        sys.exit(1)

    # Scrive le modifiche sul file
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)
        
    print(f"File '{file_path}' aggiornato:")
    print(f"   - Modalità: {args.mode.upper()}")
    if args.mode == 'real':
        print(f"   - Robot IP: {args.robot_ip}")
        print(f"   - Reverse IP: {args.reverse_ip}")

if __name__ == "__main__":
    main()