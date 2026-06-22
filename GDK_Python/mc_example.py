#!/usr/bin/env python3
import os
import sys
import agibot_gdk
import json
import glob
import time
from typing import List,Dict,Any

try:

    # Windows

    import msvcrt
    def getch() -> str:
        return msvcrt.getch().decode(sys.stdout.encoding)


except ImportError:

    # Linux / macOS

    import tty, termios
    def getch() -> str:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:

            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch

class JointPositionControl:
    def __init__(self,robot):
        self._step = 0.1
        self._speed = 0.3
        self._joint_size = 8
        self._joint_index = 0
        self._file_index = 0
        self._play_index = 0
        self._robot = robot
        self._joint_names= [
        "idx01_body_joint1",
        "idx02_body_joint2",
        "idx03_body_joint3",
        "idx04_body_joint4",
        "idx05_body_joint5",
        "idx11_head_joint1",
        "idx12_head_joint2",
        "idx13_head_joint3",
        "idx21_arm_l_joint1",
        "idx22_arm_l_joint2",
        "idx23_arm_l_joint3",
        "idx24_arm_l_joint4",
        "idx25_arm_l_joint5",
        "idx26_arm_l_joint6",
        "idx27_arm_l_joint7",
        "idx61_arm_r_joint1",
        "idx62_arm_r_joint2",
        "idx63_arm_r_joint3",
        "idx64_arm_r_joint4",
        "idx65_arm_r_joint5",
        "idx66_arm_r_joint6",
        "idx67_arm_r_joint7"]
        self._joint_positions = [0.0] * len(self._joint_names)
        self.get_json_file()

    def publish_control_request(self,target_joints,target_positions):
        joint_control_request = agibot_gdk.JointControlReq()
        joint_control_request.life_time = 1.0
        joint_control_request.joint_names = target_joints
        joint_control_request.joint_positions = target_positions
        joint_control_request.joint_velocities = [self._speed]
        robot.joint_control_request(joint_control_request)


    def adjust_joint_position(self,step):
        joint_states = robot.get_joint_states()
        self._joint_positions = [state['motor_position'] for state in joint_states['states']]
        target_position = [self._joint_positions[self._joint_index] + step]
        joint_name = [self._joint_names[self._joint_index]]
        print(f"{self._joint_names[self._joint_index]} move to {target_position}")
        self.publish_control_request(joint_name,target_position)

    def get_json_file(self):
        files = glob.glob(os.path.join("saved_commands", "*.json"))
        if not files:
            raise RuntimeError(f"No json found")
        self._files = sorted(files)
        self._file_names = [os.path.basename(p) for p in self._files]

        self._cmds: List[List[Dict[str, Any]]] = []
        for fp in files:
            with open(fp, encoding="utf-8") as f:
                data = json.load(f)
            self._cmds.append(data.get("recorded_commands", []))

    def switch_file(self):
        self._file_index = (self._file_index+1)% len(self._cmds)
        print(f"switch file to {self._file_names[self._file_index]}")
        self._play_index = 0


    def play_record(self):
        target_positions = self._cmds[self._file_index][self._play_index]['joint_positions']
        target_joints = self._cmds[self._file_index][self._play_index]['joint_names']
        self.publish_control_request(target_joints,target_positions)
        print(f"play record sequence : {self._play_index}")
        self._play_index = (self._play_index+1)%len(self._cmds[self._file_index])

    def read_key(self):
        while True:
            key = getch()
            if key == 'q' or key == 'Q':
                break
            elif key == 'a' or key =='A':
                self._joint_index =(self._joint_index-1)% len(self._joint_names)
                print(f"switch joint to {self._joint_names[self._joint_index]}")
            elif key == 'd' or key =='D':
                self._joint_index =(self._joint_index+1)% len(self._joint_names)
                print(f"switch joint to {self._joint_names[self._joint_index]}")
            elif key == 'w'or key == 'W':
                self.adjust_joint_position(self._step)
            elif key == 's'or key == 'S':
                self.adjust_joint_position(-self._step)
            elif key == 'p' or key =='P':
                self.play_record()
            elif key == 'm' or key =='M':
                self.switch_file()
            else:
                print("press 'w/s' to control the joint\npress 'a/d' to switch the joint\npress 'p' to play the record actions\npress 'm' to switch actions\npress 'q' to exit")




if __name__ == '__main__':
    agibot_gdk.gdk_init()
    robot = agibot_gdk.Robot()
    time.sleep(1)
    try:
        joint_position_control = JointPositionControl(robot)
        joint_position_control.read_key()

    except Exception as e:
        print(f"程序运行出错: {e}")
    finally:
        print("\nexit")
