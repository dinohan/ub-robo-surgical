from math import pi,radians, degrees
import tkinter as tk
import threading
from pynput import keyboard
import serial
from enum import Enum
import time
from robodk.robolink import *
from robodk.robomath import *

# HW Constants
ARDUINO_PORT = 'COM3'
ARDUINO_BAUDRATE = 115200
ARDUINO_TIMEOUT = 1
UR5e_IP = '192.168.1.5'
UR5e_PORT = 30000
UR5e_USER = 'anonymous'

# SW Constants
ZERO_YAW = 0 #To define the initial gripper orientation
# Keyboard Global flags
exit_flag = False
current_key = None
# Global variables to keep track of key states
key_states = {
    'u': False,
    'd': False,
    'e': False,
    'q': False
}

# UR5e connection online with roboDK
def Robot_online(RDK, robot, online):
    RUN_ON_ROBOT = online
    if RUN_ON_ROBOT:
        robot.setConnectionParams(UR5e_IP, UR5e_PORT, '/', UR5e_USER, '')
        time.sleep(5)
        success = robot.ConnectSafe(UR5e_IP)  # Try to connect once
        time.sleep(5)
        status, status_msg = robot.ConnectedState()
        if status != ROBOTCOM_READY:  # Stop if the connection did not succeed
            raise Exception("Failed to connect: " + status_msg)
        RDK.setRunMode(RUNMODE_RUN_ROBOT)
        print("Connection to UR5e Successful!")
        return True
    else:
        RDK.setRunMode(RUNMODE_SIMULATE)
        print("Simulation!")
        return False

# Initialize Arduino
def initialize_arduino():
    try:
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=ARDUINO_TIMEOUT)
        time.sleep(2)  # Allow some time for the connection to establish
        print(f"Arduino connected on {ARDUINO_PORT}")
        return arduino
    except serial.SerialException as e:
        print(f"Failed to connect to Arduino on {ARDUINO_PORT}")
        #print(f"Output ERROR: {e}")
        return None

# Initialize RoboDK
def initialize_robodk():
    RDK = Robolink()
    robot = RDK.Item("UR5e")
    base = RDK.Item("UR5e Base")
    endowrist = RDK.Item('Endowrist')
    gripper = RDK.Item('Gripper')
    needle = RDK.Item('Needle')
    Init_target = RDK.Item('Init')
    robot.setPoseFrame(base)
    robot.setPoseTool(endowrist)
    gripper_init = TxyzRxyz_2_Pose([0, 5, -105, 0, 0, 0])
    gripper.setParent(endowrist)
    gripper.setPose(gripper_init)
    needle_init = TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0])#YPR in RAD
    needle.setParent(gripper)
    needle.setPose(needle_init)
    robot.MoveL(Init_target)
    robot.setSpeed(5)
    return RDK, robot, base, endowrist, gripper, needle, Init_target

# Define commands
class Command(Enum):
    GET_RPW = b'\x01'

# Handle keyboard input: only alphanumeric keys are allowed
def on_press(key):
    global key_states
    try:
        if key.char in key_states:
            key_states[key.char] = True
    except AttributeError:
        pass  # Handle special keys (non-alphanumeric)

def on_release(key):
    global key_states
    try:
        if key.char in key_states:
            key_states[key.char] = False
    except AttributeError:
        pass

def update_text_label(label, text):
    # Update Tkinter Label on the main thread
    label.after(0, lambda: label.config(text=text))

def suture(arduino, robot, gripper, needle, base, text_label):
    global R, P, W  # Global variables to track rotations
    arduino.reset_input_buffer()
    
    while True:
        # Read ESP32 computer data
        arduino.write(Command.GET_RPW.value)
        g_roll, g_pitch, g_yaw = [float(arduino.readline().strip()) for _ in range(3)]
        s11, s12 = [bool(int(arduino.readline().strip())) for _ in range(2)]
        t_roll, t_pitch, t_yaw = [float(arduino.readline().strip()) for _ in range(3)]
        s21, s22 = [bool(int(arduino.readline().strip())) for _ in range(2)]
        torque_values = [float(arduino.readline().strip()) for _ in range(4)]

        endowrist_pose = robot.Pose()
        Xr, Yr, Zr, rr, pr, yr = Pose_2_TxyzRxyz(endowrist_pose)
        gripper_pose = gripper.Pose()
        Xg, Yg, Zg, rg, pg, yg = Pose_2_TxyzRxyz(gripper_pose)

        R, P, W = map(math.radians, [g_roll, g_pitch, g_yaw])
        t_R, t_P, t_W = map(math.radians, [t_roll, t_pitch, t_yaw])
        endowrist_pose = transl(Xr, Yr, Zr) * rotz(ZERO_YAW) * rotz(t_W) * roty(t_P) * rotx(t_R)

        if s11 and not s12:
            needle.setParentStatic(gripper)
            update_text_label(text_label, "Close Gripper")
        elif not s11 and not s12:
            gripper_pose = transl(Xg, Yg, Zg) * rotz(W) * roty(P) * rotx(R)
            gripper.setPose(gripper_pose)
            update_text_label(text_label, f"Mode 2. Gripper orientation: R={round(g_roll)} P={round(g_pitch)} W={round(g_yaw)}")
        elif not s11 and s12:
            needle.setParentStatic(base)
            update_text_label(text_label, "Open Gripper")
        elif key_states['u']:
            robot.MoveL(robot.Pose() * transl(0, 0, 10), True)
            key_states['u'] = False  # Reset the state after moving
            update_text_label(text_label, "Endowrist moved up")
        elif key_states['d']:
            robot.MoveL(robot.Pose() * transl(0, 0, -10), True)
            key_states['d'] = False  # Reset the state after moving
            update_text_label(text_label, "Endowrist moved down")
        elif key_states['e']:
            endowrist_pose = transl(Xr,Yr,Zr) * rotz(ZERO_YAW) * rotz(t_W) * roty(t_P) * rotx(t_R)
            if robot.MoveL_Test(robot.Joints(), endowrist_pose) == 0:
                robot.MoveL(endowrist_pose, True)
                update_text_label(text_label, f"Mode 1. Robot orientation: R={round(g_roll)} P={round(g_pitch)} W={round(g_yaw)}")
                key_states['e'] = False  # Reset the state after moving
            else:
                update_text_label(text_label, "Mode 1. Robot orientation: Robot cannot reach the position")
                key_states['e'] = False  # Reset the state after moving
        else:
            update_text_label(text_label, f"Waiting: R={round(g_roll)} P={round(g_pitch)} W={round(g_yaw)}")

# Main function
def main():
    global root
    arduino = initialize_arduino()
    if arduino is not None:
        RDK, robot, base, endowrist, gripper, needle, Init_target = initialize_robodk()
        UR5e_connected = Robot_online(RDK, robot, online=False)

        root = tk.Tk()
        root.title("Suture Process")
        text_label = tk.Label(root, text="", wraplength=300)
        text_label.pack(padx=20, pady=20)

        # Start the suture process in a separate thread
        suture_thread = threading.Thread(target=suture, args=(arduino, robot, gripper, needle, base, text_label))
        suture_thread.daemon = True
        suture_thread.start()

        # Keyboard listener in main thread
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        
        root.mainloop()
        listener.stop()  # Ensure the listener stops when the Tkinter window is closed
        print("Suture process CLOSED!")
        print("Disconnecting Arduino...")
        arduino.close()
            
        if UR5e_connected:
            print("Disconnecting UR5e...")
            robot.Disconnect()
    else:
        print("Arduino is not properly connected!")
        print("Verify Port and Hardware connection")
        print("Program CLOSED")
        
if __name__ == "__main__":
    main()
