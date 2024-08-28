# Python User Interface for Dingo-D Robot
This project involved developing a Python-based interface for teleoperating the Dingo-D robot with a Kinova arm. The interface includes manual and semi-automatic modes, allowing direct control via keyboard or target setting on a map for autonomous navigation. It also provides live video feedback for monitoring

![image](https://github.com/user-attachments/assets/3d57fa27-1428-4e41-9e5f-0cffe51c5a40)

# Camera and Arm Control Section in the GUI
In this section, the user begins by selecting the operating mode, either simulation or real robot. The user can then monitor the robot's surroundings through the onboard camera, with the option to enable or disable the camera feed. If an AprilTag is detected in the robot's environment, a red rectangle will appear in the top left corner of the GUI to indicate its presence. Additionally, the user can position the robot's arm by selecting one of the predefined positions using the available buttons. However, the interface does not provide feedback to confirm whether the arm has successfully moved to the selected position.
![image](https://github.com/user-attachments/assets/6f083b4b-da51-4b0f-8521-c72831acfbd6)

# Map of the environment and trajectory calculation in the GUI
In this section, various elements within the environment are displayed on the map based on odometry measurements. The robot is represented by a green dot, with its orientation indicated by a red arrow. Detected tags are automatically added to the map as blue dots, with their respective tag IDs displayed. The target point, marked by a red dot, only appears after the user clicks within a reachable area on the map in the GUI.

The time required to calculate the trajectory depends on the target's location, which is why a progress bar is provided below the map to indicate the calculation progress. Additionally, if a tag is detected, the user receives instructions to align the robot in front of the tag. The interface also includes a legend for the map and a guide on using keyboard keys to manually control the robot.
![image](https://github.com/user-attachments/assets/c5125a47-98be-43b0-954f-3dd535ec28e3)

