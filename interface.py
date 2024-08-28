import tkinter as tk
from tkinter import ttk
import numpy as np
import threading 
import signal
import sys
import rospy
from os import environ
import math
from std_msgs.msg import Float32MultiArray
from jackal_msgs.msg import Drive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from mobile_manip.srv import ReachName, GetValues, ReachValues
from scipy.spatial.transform import Rotation as R
from PIL import Image, ImageTk
from lab_utils.plan_utils import *
from lab_utils.astart import AStarPlanner
from path_planning import initialize_ros_publishers_and_subscribers, ground_truth_callback, imu_callback, odometry_callback, cam_callback
from path_planning import draw_robot, image_to_matrix, on_click, on_key_press, follow_path, get_last_calculated_path, read_SIM_value
from camera import camera_view, update_tag_detection, add_tag_to_map, robot_close_to_tag, print_tag_info, tag_detected
from robot_arm import reach_coords, reach_recorded_home, reach_recorded_vertical, reach_recorded_retract

# Team and robot number
NUMERO_EQUIPE = 7
NUMERO_ROBOT = 4

SIM = read_SIM_value()
sim = None
rate = None
root = None
camera = None

# Check if running in simulation mode
if SIM:
    print("SIMULATION")
    pass
else:
    print("ROBOT REEL")
    environ['ROS_MASTER_URI'] = "http://cpr-ets05-0{}.local:11311/".format(NUMERO_ROBOT)
    environ['ROS_IP'] = "192.168.0.84"

def run_ros_node():
    global rate, root

    # Create and start a new ROS node
    rospy.init_node('dingo_controller', anonymous=True, disable_signals=True)
    rospy.loginfo("ROS Node has been started")
    rate = rospy.Rate(50) # Set loop rate to 50 Hz

    # Initialize ROS publishers and subscribers
    initialize_ros_publishers_and_subscribers()

    # Keep the node running until the GUI is shut down
    while not rospy.is_shutdown():
        rate.sleep()

def create_ros_thread():
    # Create a thread to run the ROS node
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.daemon = True
    ros_thread.start()


def run_interface():
    global root, camera, sim, SIM 

    # Create a thread to run the ROS node
    create_ros_thread()

    root = tk.Tk() # Create the main Tkinter window
    root.title("Interface de contrôle")

    # Set window size
    screen_width = 1400
    screen_height = 1000
    root.geometry(f"{screen_width}x{screen_height}")

    # Configure grid layout for window sections
    root.grid_columnconfigure(0, weight=1)
    root.grid_columnconfigure(1, weight=2)  # Middle column where section 1 and 2 are placed
    root.grid_columnconfigure(2, weight=1)
    root.grid_rowconfigure(0, weight=1)
    root.grid_rowconfigure(1, weight=2)  # Middle row where section 1 and 2 are placed 
    root.grid_rowconfigure(2, weight=1)

    # Global variable for simulation or real mode
    sim = tk.StringVar(value='Simulation')  # Initial mode

    # Global variable to manage camera view state
    camera = tk.BooleanVar(value=False)

    # Function to switch between simulation and real mode
    def change_mode():
        if sim.get() == "Simulation":
            sim.set("Réel")
            SIM = False
            print("SIM apres changement : ", SIM)
        else:
            sim.set("Simulation")
            SIM = True
            print("SIM apres changement : ", SIM)

        text_mode.config(text="Mode actuel : " + sim.get())

        # Save the mode setting to a file
        with open("sim_config.txt", "w") as f:
            f.write(str(SIM))

    def toggle_camera_view(value):
        # If the value is 1, enable camera view
        if int(value) == 1:
            camera.set(True)
            camera_view_text.config(text="Caméra : Activée")
            print("Vue de la caméra activée")
        else:
            camera.set(False)
            camera_view_text.config(text="Caméra : Désactivée")
            print("Vue de la caméra désactivée")  

    # SECTION 1: TOP-CENTERED
    section1_width = screen_width // 4
    section1_height = screen_height // 3
    section1 = tk.Frame(root, bg='#D7E8FA', width=section1_width, height=section1_height)
    section1.grid_propagate(False)
    section1.grid(row=0, column=1, sticky="nsew")

    # Set column and row weights for section 1
    section1.grid_columnconfigure(0, weight=1)
    section1.grid_rowconfigure(0, weight=0) 
    section1.grid_rowconfigure(1, weight=1)

    label = tk.Label(section1, bg='black') # Create a label to display the camera view
    label.grid(row=1, column=0, sticky="nsew")

    # Create a label to display the current mode
    text_mode = tk.Label(section1, text="Mode actuel : " + sim.get(), font=("Roboto", 12), bg='#D7E8FA', fg='black')
    text_mode.grid(row=0, column=0, sticky="ew")

    # Create a button to switch modes
    button_mode = tk.Button(section1, text="Changer de mode", font=("Roboto", 12), command=change_mode, bg='#ffffff', fg='#000000')
    button_mode.grid(row=0, column=1, sticky="ne", padx=10, pady=5)
    button_mode.configure(borderwidth=2, relief="raised")

    camera_view_text = tk.Label(section1, text="Caméra : Désactivée", font=("Roboto", 12), bg='#D7E8FA', fg='black')
    camera_view_text.grid(row=0, column=1, sticky="ne", padx=10, pady=(35, 0)) 

    # Create a slider to enable/disable the camera view
    camera_view_slider = tk.Scale(section1, from_=0, to=1, orient="horizontal", showvalue=0, sliderlength=50, length=100, fg='white', troughcolor='white', command=toggle_camera_view)
    camera_view_slider.grid(row=0, column=1, sticky="ne", padx=10, pady=(60, 0))

    def camera_update():
        if not root:  # Stop the function if the window is closed
            return

        if camera.get(): # If camera view is enabled
            image = camera_view(section1_width, section1_height)
            if image:
                update_tag_detection(image)
                imageTk = ImageTk.PhotoImage(image)  
                label.configure(image=imageTk)
                label.image = imageTk
        else :
            label.configure(image=None)
            label.image = None
        
        if root:
            root.after(100, camera_update)  # Update the camera view every 100 ms

    camera_update() 
    
    # Start a thread to update the camera view
    camera_thread = threading.Thread(target=camera_update) 

    # Change the button color on hover
    def on_enter(e):
        button_mode['background'] = '#abcdef'  # Hover background color
        button_mode['foreground'] = '#ffffff'  # Hover text color

    def on_leave(e):
        button_mode['background'] = '#ffffff'  # Original background color
        button_mode['foreground'] = '#000000'  # Original text color

    # Bind hover events to the button
    button_mode.bind("<Enter>", on_enter)
    button_mode.bind("<Leave>", on_leave)

    # Place the button in the top-right corner of section 1
    button_mode.grid(row=1, column=0, sticky="ne", padx=10, pady=5)
    button_mode.configure(borderwidth=2, relief="raised")

    # Create a frame for robot control buttons
    button_frame = tk.Frame(section1, bg='#D7E8FA')
    button_frame.grid(row=1, column=1, sticky="nsew", padx=10, pady=5)
    button_frame.grid_columnconfigure(0, weight=1)
    button_frame.grid_columnconfigure(1, weight=3)  
    button_frame.grid_rowconfigure(list(range(7)), weight=1)

    # Display text to indicate arm control
    text_control = tk.Label(button_frame, text="Contrôle du bras", font=("Roboto", 12), bg='#D7E8FA', fg='black')
    text_control.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

    # Define buttons for predefined positions
    button_texts = ["Go to Home", "Go to Vertical", "Go to Retract"]
    commands = [reach_recorded_home, reach_recorded_vertical, reach_recorded_retract]

    go_to_position_button = tk.Button(button_frame, text="Go to tag", font=("Roboto", 12), command=reach_coords, bg='#ffffff', fg='#000000')
    go_to_position_button.grid(row=1, column=0, columnspan=2, sticky="ew", padx=5, pady=2)
    go_to_position_button.configure(borderwidth=2, relief="raised")

    for i, (text, cmd) in enumerate(zip(button_texts, commands), start=2):
        button = tk.Button(button_frame, text=text, font=("Roboto", 12), command=cmd, bg='#ffffff', fg='#000000')
        button.grid(row=i, column=0, columnspan=2, sticky="ew", padx=5, pady=2)
        button.configure(borderwidth=2, relief="raised")

    # SECTION 2: CENTERED
    section2_height = screen_height // 2
    section2_width = screen_width // 2
    section2 = tk.Frame(root, bg='#D7E8FA', width=section2_width, height=section2_height)
    section2.grid_propagate(False)

    section2.grid(row=1, column=1, sticky="nsew")
    section2.grid_columnconfigure(0, weight=3)  
    section2.grid_columnconfigure(1, weight=1)  
    section2.grid_rowconfigure(0, weight=1)

    image_path = "a2230_map_closed_fliped.png" # Path to the map image
    original_img = Image.open(image_path)
    original_width, original_height = original_img.size
    resized_img = original_img.resize((section2_width, section2_height), Image.LANCZOS)
    img = ImageTk.PhotoImage(resized_img)

    # Create a canvas to display the image
    canvas = tk.Canvas(section2, width=section2_width//2, height=section2_height, bg='#D7E8FA', highlightthickness=0)
    canvas.create_image(0, 0, image=img, anchor="nw")
    canvas.grid(row=0, column=0, rowspan=2, sticky="nsew")

    # Add a green dot to represent the robot's position
    robot = canvas.create_oval(0, 0, 10, 10, fill="green", tags="robot")  
    arrow = canvas.create_line(0, 0, 0, 0, arrow=tk.LAST, fill="red", tags="arrow")
    draw_robot(canvas, robot, arrow, section2_width, section2_height, original_width, original_height)

    # Add detected tags to the 2D map
    add_tag_to_map(canvas, original_img, section2_width, section2_height, original_width, original_height)

    # Add a progress bar for the A* algorithm
    progress = ttk.Progressbar(section2, orient="horizontal", length=300, mode="determinate")
    progress.grid(row=2, column=0)

    # Display text indicating that the trajectory calculation is in progress
    text_progress = tk.Label(section2, text="Calcul de la trajectoire en cours...", font=("Roboto", 12), bg='#D7E8FA', fg='black')
    text_progress.grid(row=1, column=0, sticky="s")

    root.bind('<KeyPress>', lambda event: on_key_press(event, canvas, section2_width, section2_height, original_width, original_height))

    # Bind the add_point function to mouse clicks
    canvas.bind('<Button-1>', lambda event: on_click(canvas, original_img, section2_width, section2_height, original_width, original_height, event, progress))

    # Add a legend for the elements on the map
    legende_container = tk.Frame(section2, bg='#D7E8FA')
    legende_container.grid(row=0, column=3,  padx=20, pady=20) 

    text_robot = tk.Label(legende_container, text="Position robot", font=("Roboto", 12), bg='#D7E8FA', fg='black')
    text_robot.grid(row=0, column=1, sticky="nw")
    canvas_robot_pos = tk.Canvas(legende_container, width=20, height=20, bg='#D7E8FA', highlightthickness=0)
    canvas_robot_pos.create_oval(5, 5, 15, 15, fill="green")  
    canvas_robot_pos.grid(row=0, column=0, sticky="nw")

    text_arrow = tk.Label(legende_container, text="Orientation robot", font=("Roboto", 12), bg='#D7E8FA', fg='black')
    text_arrow.grid(row=1, column=1, sticky="nw")
    canvas_arrow = tk.Canvas(legende_container, width=20, height=20, bg='#D7E8FA', highlightthickness=0)
    canvas_arrow.create_line(5, 10, 15, 10, arrow=tk.LAST, fill="red")  
    canvas_arrow.grid(row=1, column=0, sticky="nw")

    text_end_point = tk.Label(legende_container, text="Position cible", font=("Roboto", 12), bg='#D7E8FA', fg='black')
    text_end_point.grid(row=2, column=1, sticky="nw")
    canvas_end_pos = tk.Canvas(legende_container, width=20, height=20, bg='#D7E8FA', highlightthickness=0)
    canvas_end_pos.create_oval(5, 5, 15, 15, fill="red")  
    canvas_end_pos.grid(row=2, column=0, sticky="nw")

    text_tag = tk.Label(legende_container, text="Position tag", font=("Roboto", 12), bg='#D7E8FA', fg='black')
    text_tag.grid(row=3, column=1, sticky="nw")
    canvas_tag = tk.Canvas(legende_container, width=20, height=20, bg='#D7E8FA', highlightthickness=0)
    canvas_tag.create_oval(5, 5, 15, 15, fill="blue")
    canvas_tag.grid(row=3, column=0, sticky="nw")

    # Display how to manually stop the robot
    text_stop = tk.Label(legende_container, text="Pour arrêter le robot : Spacebar", font=("Roboto", 12), bg='#D7E8FA', fg='black')
    text_stop.grid(row=4, column=0, columnspan=2, sticky="nw")

    # Add empty rows to separate sections
    tk.Label(legende_container, text="", bg='#D7E8FA').grid(row=5, column=0, columnspan=2)
    tk.Label(legende_container, text="", bg='#D7E8FA').grid(row=6, column=0, columnspan=2)

    # Display dynamic information about the tag
    text_dynamic = tk.Label(legende_container, text="Informations dynamiques tag :", font=("Roboto", 12), bg='#D7E8FA', fg='black')
    text_dynamic.grid(row=7, column=0, columnspan=2, sticky="nw")

    detect_tag_text = tk.StringVar(value="Etat : Tag non détecté")
    detect_tag = tk.Label(legende_container, textvariable=detect_tag_text, font=("Roboto", 12), bg='#D7E8FA', fg='black')
    detect_tag.grid(row=8, column=0, columnspan=2, sticky="nw")

    def update_detect_tag_text() :
        if tag_detected():
            detect_tag_text.set("Etat : Tag détecté")
            detect_tag.config(fg='green')
        else:
            detect_tag_text.set("Etat : Tag non détecté")
            detect_tag.config(fg='red')

        root.after(1000, update_detect_tag_text)

    update_detect_tag_text()

    # Global variable to manage robot proximity to the tag
    proximity_text = tk.StringVar(value="Robot éloigné du tag")
    proximity_label = tk.Label(legende_container, textvariable=proximity_text, font=("Roboto", 12), bg='#D7E8FA', fg='black')
    proximity_label.grid(row=9, column=0, columnspan=2, sticky="nw")

    tag_text = tk.StringVar()
    tag_label = tk.Label(legende_container, textvariable=tag_text, font=("Roboto", 12), bg='#D7E8FA', fg='black', wraplength=180)
    tag_label.grid(row=10, column=0, columnspan=2, rowspan=2, sticky="nw")
    
    def update_proximity_text() : 
        close_to_tag = robot_close_to_tag()
        if close_to_tag : 
            proximity_text.set("Robot proche du tag")
            proximity_label.config(fg='green')
            if print_tag_info() == 0:
                tag_text.set("Tourner à droite pour centrer le robot en face du tag")
                tag_label.config(fg='blue')
            elif print_tag_info() == 1:
                tag_text.set("Tourner à gauche pour centrer le robot en face du tag")
                tag_label.config(fg='blue')
            if print_tag_info() == 2:
                tag_text.set("En face du tag")
                tag_label.config(fg='blue')
        else :
            proximity_text.set("Robot éloigné du tag")
            proximity_label.config(fg='red')
            if tag_detected():
                if print_tag_info() == 0:
                    tag_text.set("Tourner à droite pour centrer le robot en face du tag")
                    tag_label.config(fg='blue')
                elif print_tag_info() == 1:
                    tag_text.set("Tourner à gauche pour centrer le robot en face du tag")
                    tag_label.config(fg='blue')
                elif print_tag_info() == 2:
                    tag_text.set("En face du tag")
                    tag_label.config(fg='blue')
            else:
                tag_text.set("")
                tag_label.config(fg='blue')

        root.after(1000, update_proximity_text)

    update_proximity_text()

    # Add instructions in section 2 to manually control the robot
    arrow_container = tk.Frame(section2, bg='#D7E8FA')
    arrow_container.grid(row=1, column=3, sticky="ew")

    # Text and arrows for movement instructions
    text_instructions = tk.Label(arrow_container, text="Touches pour contrôler \n manuellement le robot", font=("Roboto", 10), bg='#D7E8FA', fg='black')
    text_instructions.grid(row=0, column=0, columnspan=3, sticky="nsew")

    arrow_up = tk.Label(arrow_container, text="↑ Up", font=("Arial", 14), bg='#D7E8FA', fg='black')
    arrow_up.grid(row=1, column=1, sticky="n")  # Align to the north

    arrow_left = tk.Label(arrow_container, text="Left\n←", font=("Arial", 14), bg='#D7E8FA', fg='black')
    arrow_left.grid(row=2, column=0, sticky="w")  # Align to the west

    arrow_down = tk.Label(arrow_container, text="↓ Down", font=("Arial", 14), bg='#D7E8FA', fg='black')
    arrow_down.grid(row=3, column=1, sticky="s")  # Align to the south

    arrow_right = tk.Label(arrow_container, text="Right\n→", font=("Arial", 14), bg='#D7E8FA', fg='black')
    arrow_right.grid(row=2, column=2, sticky="e")  # Align to the east

    root.mainloop()

def on_sigint(signal, frame):
   
    rospy.signal_shutdown("GUI shutdown")
    
    # Shutdown ROS and close the GUI on Ctrl+C
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, on_sigint)
    
    run_interface()

    print("Shutting down safely...")