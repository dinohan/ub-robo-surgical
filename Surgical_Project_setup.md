# Python Project setup

This "Surgical Robotics Project" consist on the development of a Suture process using the UB Surgical Robotic System.

The Project will be structured in 1 Activity and 4 Laboratory Sessions:
- A3: Suture surgery Simulation
- Lab Session 1
- Lab Session 2
- Lab Session 3
- Lab Session 4

Let's consider the Objectives of the Activity 3 and the lab sessions will be explained every week in the Virtual Campus.

## **A3: Suture surgery Simulation**

First of all, download the project folder "Surgery_students2024" in VS code and follow the instructions below:
- Review the "settings.json" to address the correct python interpreter in terminal
- For windows users, consider adding this directory to PATH (not necessary): 'C:\RoboDK\Python-Embedded\Scripts'
- update the pip version within the selected python interpreter:
    ````shell
    python -m pip install --upgrade pip
    ````
- Install the needed packages (ensuring you are using the correct pip version associated to the choosen python environment in settings.json file)::
    ````shell
    python -m pip install pynput
    python -m pip install spatialmath-python
    ````
- Open "Suture_students.rdk" robodk file from Windows FileManager
- Open "Suture_sw_proposal.py" python file from VS code.
    - Review the program structure, 
    - RUN the program to execute a simple suture process
    - Plan a better suture process according to the DaVinci suture video exemples 
    - Modify the program and generate the "Suture_sw_custom.py"
    - Identify and comment the main problems to perform the suture

Upload the:
- generated "Suture_sw_custom.py" 
- a document with the main problems you have found 

This activity has to be developped in Laboratory subgroups