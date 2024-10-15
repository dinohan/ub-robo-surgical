from math import pi, radians, degrees
import tkinter as tk
import threading
from pynput import keyboard
import time
from robodk.robolink import *
from robodk.robomath import *

# SW Constants in RAD
ZERO_YAW = 0  # To define the initial gripper orientation
R = 0
P = 0
W = 0

# Global variables to keep track of key states
key_states = {
    'c': False,
    'o': False,
    'u': False,
    'd': False,
    'e': False,
    'g': False
}

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
    needle_init = TxyzRxyz_2_Pose([0, 0, 0, 0, pi/2, pi/2])  # YPR in RAD
    needle.setParent(gripper)
    needle.setPose(needle_init)
    robot.MoveL(Init_target)
    robot.setSpeed(5)
    return RDK, robot, base, endowrist, gripper, needle, Init_target

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

# Tkinter label update
def update_text_label(label, text):
    label.after(0, lambda: label.config(text=text))

# Suture process function
def suture(robot, gripper, needle, base, text_label):
    global R, P, W  # Global variables to track rotations

    while True:
        # Read the position of the robot and the gripper
        endowrist_pose = robot.Pose()
        Xr, Yr, Zr, rr, pr, yr = Pose_2_TxyzRxyz(endowrist_pose)
        gripper_pose = gripper.Pose()
        Xg, Yg, Zg, rg, pg, yg = Pose_2_TxyzRxyz(gripper_pose)

        endowrist_pose = transl(Xr, Yr, Zr) * rotz(ZERO_YAW) * rotz(W) * roty(P) * rotx(R)

        # Close Gripper: Needle nested to Gripper - 'c' key
        if key_states['c']:
            needle.setParentStatic(gripper)
            update_text_label(text_label, "Close Gripper")
            key_states['c'] = False  # Reset the state after action
            time.sleep(1)

        # Move Gripper - 'g' key
        elif key_states['g']:
            R += 1 * pi / 180
            gripper_pose = transl(Xg, Yg, Zg) * rotz(W) * roty(P) * rotx(R)
            gripper.setPose(gripper_pose)
            update_text_label(text_label, f"Move Gripper: R={round(degrees(R))} P={round(degrees(P))} W={round(degrees(W))}")
            key_states['g'] = False
            time.sleep(1)

        # Open Gripper: Needle nested to Robot - 'o' key
        elif key_states['o']:
            needle.setParentStatic(base)
            update_text_label(text_label, "Open Gripper")
            key_states['o'] = False
            time.sleep(1)

        # Move robot up - 'u' key
        elif key_states['u']:
            robot.MoveL(robot.Pose() * transl(0, 0, 10), True)
            update_text_label(text_label, "Move Endowrist Up")
            key_states['u'] = False
            time.sleep(1)

        # Move robot down - 'd' key
        elif key_states['d']:
            robot.MoveL(robot.Pose() * transl(0, 0, -10), True)
            update_text_label(text_label, "Move Endowrist Down")
            key_states['d'] = False
            time.sleep(1)

        # Rotate robot - 'e' key
        elif key_states['e']:
            R += 1 * pi / 180
            endowrist_pose = transl(Xr, Yr, Zr) * rotz(ZERO_YAW) * rotz(W) * roty(P) * rotx(R)
            if robot.MoveL_Test(robot.Joints(), endowrist_pose) == 0:
                robot.MoveL(endowrist_pose, True)
                update_text_label(text_label, f"Rotate Robot: R={round(degrees(R))} P={round(degrees(P))} W={round(degrees(W))}")
            else:
                update_text_label(text_label, "Cannot reach position")
            key_states['e'] = False
            time.sleep(1)

        # Default waiting state
        else:
            update_text_label(text_label, f"Waiting: R={round(degrees(R))} P={round(degrees(P))} W={round(degrees(W))}")

# Main function
def main():
    global root
    # Initialize RoboDK and Tkinter
    RDK, robot, base, endowrist, gripper, needle, Init_target = initialize_robodk()
    # Init Keyboard check with TKinter
    root = tk.Tk()
    root.title("Suture Process")
    text_label = tk.Label(root, text="", wraplength=300)
    text_label.pack(padx=20, pady=20)

    # Start the suture process in a separate thread
    suture_thread = threading.Thread(target=suture, args=(robot, gripper, needle, base, text_label))
    suture_thread.daemon = True
    suture_thread.start()

    # Keyboard listener in main thread
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    root.mainloop()
    listener.stop()  # Ensure the listener stops when the Tkinter window is closed
    print("Suture process CLOSED!")

if __name__ == "__main__":
    main()
