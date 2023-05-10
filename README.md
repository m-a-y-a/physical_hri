# physical_hri

This project aimed to program the TIAGo robot to assist people with limited mobility with their daily tasks by retrieving objects that are verbally requested. This project was completed in Python and contains subcomponents such as speech, navigation, object detection, and arm movement.

## To run our code:

1. Clone this directory on TIAGo's operating system - "git clone https://github.com/m-a-y-a/physical_hri.git"
2. Navigate into the scripts directory - "cd physical_hri/scripts/"
	- The code is all contained withing run_tiago.py
3. To run the code, use the command "python run_tiago.py" while in the scripts directory

 ## Program Architecture

The run_tiago.py file creates a class called run_tiago and instantiates all ROS publishers, subscribers, services, action clients, and global variables, such as locations, upon execution of the main method. It contains the following pipelines:

**Main method:**
- run() - lines 136-162: The initial base movement code (navigation pipeline) moves TIAGo to in front of the inventory table to read and process the positions of the AruCo tags (object detection pipeline). The TIAGo base is moved to the center of the room to begin the interaction with the user (speech pipeline). 

**Navigation Pipeline:**
- move_to() - lines 437 - 494: Takes an x, y location in the global coordinate system that the robot should move to or a degrees (180, 90, 0, -90 being the four sides of the project space) that the robot sould turn towards. Calculates the distance to move/radians to turn by subtracting the robot's current location from the location it has to move to. This method calls move_base().
- move_base() - lines 496 - 549: Takes the distance to move or radians to turn, calculates the seconds it would take to move that distance given the preset speed of 0.2 m/s or 0.4 rad/s and adjusts the speed using the rounded time since the base controller only takes integer time values.
    
**Object Detection:**
- move_head_to_position() - lines 690 - 726: Uses a simple action client with a follow joint trajectory goal to move the head down so it can see all of the markers for AruCo marker detection. This method calls keep_head_still()
- keep_head_still() - lines 668 - 688: Starts the head manager publisher to disable the constant head motion that TIAGo does in order to get a static image for AruCo detection
- save_pose() - lines 728-742: Takes the marker pose array returned by the AruCo detection node and store it as a class variable for access in other code areas
- get_marker_pose() - lines 744-766: Returns the x, y, and z position of the specific AruCo marker (the marker with the given ID) relative to TIAGo's base

**Speech Pipeline:**
- send_cmd() - line 600-666: The Google speech recognition API is used here to detect whichever command the user gives. The user may choose to engage in a short conversation with the robot (using the PAL Text-to-Speech) and introduce themselves or they may request an item from the table. This method calls do_cmd() to bring the requested object. The user may then either thank the robot or tell it that the wrong item was retrieved, to which TIAGo will either express "You're welcome" or remorse for making a mistake. The TIAGo base is then moved back to the center of the space to await further requests of interactions.
- do_cmd() - lines 311-435: If an item is requested, TIAGo's base is moved in front of the inventory table and its torso is raised while its arm is extended. The base is then moved to align the hand with the requested object's AruCo marker position and the grasp service proxy is used to grab the item. The base is then moved to in front of the dropoff table location, the torso is lowered to the table height, and the grasp service proxy opens the hand to release the item (arm movement pipeline).

**Arm Movement:**
- move_torso() - lines 203 - 232: Moves TIAGo's torso to a specific height using a simple action client that takes a follow trajectory point.
- move_arm() - lines 234 - 265:  Moves TIAGo's right arm joints to a specific positions using a simple action client that takes a follow trajectory point.
- play_motion() - lines 177 - 201: Takes the name of a preset motion that TIAGo can do and plays it using the motion goal simple action client.
    

A visual representation of our code is included below:
![code_flowchart](https://github.com/m-a-y-a/physical_hri/assets/43100445/35c5dbcb-5c9b-45c7-8cc2-3802d993789e)
