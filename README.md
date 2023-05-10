# physical_hri

This project aimed to program the TIAGo robot to assist people with limited mobility with their daily tasks by retrieving objects that are verbally requested. This project was completed in Python and contains subcomponents such as speech, navigation, object detection, and arm movement.

## To run our code:

1. Clone this directory on TIAGo's operating system - "git clone https://github.com/m-a-y-a/physical_hri.git"
2. cd physical_hri/scripts/
	- The code is all contained withing run_tiago.py
3. To run the code, use the command "python run_tiago.py" while in the scripts directory

 ## Program Architecture

The run_tiago.py file creates a class called run_tiago and instantiates all ROS publishers, subscribers, services, action clients, and global variables, such as locations, upon execution of the main method. It then completes the following tasks in succession:

1. run() - lines 136-162: The initial base movement code moves TIAGo to in front of the inventory table to read and process the positions of the AruCo tags
2. run() - lines 164-174: The TIAGo base is moved to the center of the room to begin the interaction with the user. 
3. send_cmd() - line 600-666: The Google speech recognition API is used here to detect whichever command the user gives. The user may choose to engage in a short conversation with the robot (using the PAL Text-to-Speech) and introduce themselves or they may request an item from the table.
4. do_cmd() - lines 311-435: If an item is requested, TIAGo's base is moved in front of the inventory table and its torso is raised while its arm is extended. The base is then moved to align the hand with the requested object's AruCo marker position and the grasp service proxy is used to grab the item. The base is then moved to in front of the dropoff table location, the torso is lowered to the table height, and the grasp service proxy opens the hand to release the item.
5. send_cmd() - line 600-666: The user may then either thank the robot or tell it that the wrong item was retrieved, to which TIAGo will either express "You're welcome" or remorse for making a mistake. The TIAGo base is then moved back to the center of the space to await further requests of interactions.

A visual reresentation of our code is included below:
![code_flowchart](https://github.com/m-a-y-a/physical_hri/assets/43100445/35c5dbcb-5c9b-45c7-8cc2-3802d993789e)
