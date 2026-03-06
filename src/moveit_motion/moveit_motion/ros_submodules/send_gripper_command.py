import subprocess
import sys

def send_gripper_command(position, max_effort):
    command = f'ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{{command: {{position: {position}, max_effort: {max_effort}}}}}"'
    result = subprocess.run(command, shell=True, capture_output=True, text=True)
    if result.returncode == 0:
        print("Command executed successfully.")
        print(result.stdout)
    else:
        print("Error executing command.")
        print(result.stderr)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python send_gripper_command.py <position> <max_effort>")
    else:
        position = float(sys.argv[1])
        max_effort = float(sys.argv[2])
        send_gripper_command(position, max_effort)
