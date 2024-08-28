import numpy as np
from lab_utils.plan_utils import *
from lab_utils.astart import AStarPlanner
import time
import asyncio
import threading
import rospy
from os import environ
import math
from std_msgs.msg import Float32MultiArray
from jackal_msgs.msg import Drive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from apriltag_ros.msg import AprilTagDetectionArray
from control_msgs.msg import FollowJointTrajectoryActionFeedback
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image

def read_SIM_value():
    try:
        with open("sim_config.txt", "r") as f:
            value = f.read().strip()
            return value.lower() in ['true', '1', 't', 'y', 'yes'] # convert to lowercase and compare
    except FileNotFoundError:
        # Default value if the file does not exist
        return True

SIM = read_SIM_value()

robot_pos = None # Robot position
end_pos = None # Target position
x_robot_ros = 0
y_robot_ros = 0
x_end_ros = 0
y_end_ros = 0
last_calculated_path = []
cmd_drive_pub = None
imu_sub = None
ground_truth_sub = None
odom_sub = None
cam_sub = None
tag_sub = None
cmd_vel_pub = None
arm_trajectory_sub = None

# Robot dimensions in [m]
interwheel_distance = 0.3765
left_wheel_radius = 0.049
right_wheel_radius = 0.049

stop_follow_path = False

# IMU callback function
imu_msg = Imu()
def imu_callback(msg):
    global imu_msg
    imu_msg = msg

x_gt = 0
y_gt = 0
theta_gt = 0
ground_truth_msg = Odometry()
# Ground truth subscriber callback
def ground_truth_callback(msg):
    global x_gt, y_gt, theta_gt, ground_truth_msg
    ground_truth_msg = msg
    x_gt = msg.pose.pose.position.x
    y_gt = msg.pose.pose.position.y
    theta_gt = get_heading_from_quaternion(msg.pose.pose.orientation)

x_uwb = 0
y_uwb = 0
theta_uwb = 0
uwb_msg = Odometry()
# Odometry subscriber callback
def odometry_callback(msg):
    global x_uwb, y_uwb, theta_uwb, uwb_msg
    uwb_msg = msg
    x_uwb = msg.pose.pose.position.x
    y_uwb = msg.pose.pose.position.y
    theta_uwb = get_heading_from_quaternion(msg.pose.pose.orientation)

# Camera subscriber callback
cam_msg = Image()
def cam_callback(msg):
    global cam_msg
    cam_msg = msg

# AprilTag subscriber callback
tag_array = AprilTagDetectionArray()
def tag_callback(msg):
    global tag_array
    tag_array = msg

# Arm trajectory subscriber callback
arm_msg = FollowJointTrajectoryActionFeedback()
def feedback_callback(msg):
    global arm_msg
    arm_msg = msg

def initialize_ros_publishers_and_subscribers():
    global cmd_drive_pub, imu_sub, ground_truth_sub, odom_sub, cam_sub, cmd_vel_pub
    # ROS Publishers et subscribers
    if SIM: # Simulation
        cmd_drive_pub = rospy.Publisher('/mobile_manip/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
        imu_sub = rospy.Subscriber('/imu/data', Imu, imu_callback)
        ground_truth_sub = rospy.Subscriber("/mobile_manip/ground_truth/state", Odometry, ground_truth_callback)
        cam_sub = rospy.Subscriber('/mobile_manip/d435i/color/image_raw', Image, cam_callback)
        tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)
        cmd_vel_pub = rospy.Publisher('mobile_manip/cmd_vel', Twist, queue_size=1)
        arm_traject_sub = rospy.Subscriber("/mobile_manip/arm_gen3_lite_joint_trajectory_controller/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback, feedback_callback)

    else: # Real Robot 
        cmd_drive_pub = rospy.Publisher('/mobile_manip/base/dingo_velocity_controller/cmd_drive', Drive, queue_size=1)
        imu_sub = rospy.Subscriber('/mobile_manip/base/imu/data', Imu, imu_callback)
        odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, odometry_callback) # UWB sensor odometry
        cam_sub = rospy.Subscriber('/mobile_manip/d435i/color/image_raw', Image, cam_callback)
        tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)
        cmd_vel_pub = rospy.Publisher('mobile_manip/base/cmd_vel', Twist, queue_size=1)
        arm_traject_sub = rospy.Subscriber("/mobile_manip/arm_gen3_lite_joint_trajectory_controller/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback, feedback_callback)

# Function to wrap angle between -pi and pi
def wraptopi(angle):
    xwrap=np.remainder(angle, 2*np.pi)
    if np.abs(xwrap)>np.pi:
        xwrap -= 2*np.pi * np.sign(xwrap)
    return xwrap

# Function to calculate orientation from a quaternion
def get_heading_from_quaternion(q):
    r = R.from_quat([q.x, q.y, q.z, q.w])
    angles = r.as_euler('xyz', degrees=False)
    return angles[2]

# Function to get the robot pose (x, y, theta) based on the simulation or the real robot
def get_robot_pos():
    if SIM:
        return x_gt, y_gt, theta_gt
    else:
        return x_uwb, y_uwb, theta_uwb

# Function to get the robot pose (x, y, theta) based on the simulation or the real robot
def convert_robot_pos(section2_width, section2_height, original_width, original_height) : 
    global robot_pos

    robot_x, robot_y, robot_theta = get_robot_pos()

    # Map resolution (in meters per pixel)
    map_res = 0.02

    # Offset to adjust 2D map coordinates to ROS map
    if SIM:
        offset_x = 0.0
        offset_y = 0.0
        cap = wraptopi(robot_theta)
    else:
        offset_x = 4.6 
        offset_y = -4.2 
        cap = wraptopi(theta_uwb)

    x_start = 23.7 + offset_x   
    y_start = 22.6 + offset_y   

    x_pos = robot_x + x_start
    y_pos = robot_y + y_start

    # Convert robot coordinates to fit image size
    canvas_robot_x = (x_pos/ map_res) * (section2_width / original_width)
    canvas_robot_y = (y_pos / map_res) * (section2_height / original_height)
    
    return canvas_robot_x, canvas_robot_y, cap

def convert_map_to_pos(x_map, y_map, section2_width, section2_height, original_width, original_height) :

    map_res = 0.02

    x_pos = (x_map * map_res) - 23.7
    y_pos = (y_map * map_res) - 22.6

    return x_pos, y_pos

def draw_robot(canvas, robot, arrow, section2_width, section2_height, original_width, original_height):
    global robot_pos

    canvas_robot_x, canvas_robot_y, cap = convert_robot_pos(section2_width, section2_height, original_width, original_height)
    # Adjust robot coordinates to fit A* algorithm
    robot_pos = Point(int(canvas_robot_x * (original_width/ section2_width)), int(canvas_robot_y * (original_height / section2_height)))  
    radius = 5  # Circle radius

    arrow_length = 20
    ang_x = canvas_robot_x + arrow_length * math.cos(cap)
    ang_y = canvas_robot_y + arrow_length * math.sin(cap)

    # Update robot position
    existing_robot = canvas.find_withtag("robot")
    if existing_robot:
        canvas.coords(existing_robot, canvas_robot_x - radius, canvas_robot_y - radius,
                      canvas_robot_x + radius, canvas_robot_y + radius)

    else:
        canvas.create_oval(canvas_robot_x - radius, canvas_robot_y - radius,
                           canvas_robot_x + radius, canvas_robot_y + radius, fill="green", tags="robot")

    existing_arrow = canvas.find_withtag("arrow")
    if existing_arrow:
        canvas.coords(existing_arrow, canvas_robot_x, canvas_robot_y, ang_x, ang_y)
    else:
        canvas.create_line(canvas_robot_x, canvas_robot_y, ang_x, ang_y, fill="red", tags="arrow", width=2)
    

def image_to_matrix(original_img):
    img_array = np.array(original_img)
    map_img = 1 - np.floor_divide(img_array[:,:,1], 255)
    mat_map = map_img
    return mat_map

def update_path(canvas, start, end, section2_width, section2_height, original_width, original_height, original_img, progress):
    global last_calculated_path
    # Initialize the progress bar
    progress["value"] = 0
    progress["maximum"] = 100 

    # Remove the previous path
    canvas.delete("path_line")

    # Initialize the map and the A* planner
    mat_map = image_to_matrix(original_img)
    map = BMPMap(width=mat_map.shape[1], height=mat_map.shape[0], mat=mat_map)
    astarPlanner = AStarPlanner(map=map, step_size=5, heuristic_dist='Euclidean')

    progress["value"] = 50  
    progress.update_idletasks()

    path = astarPlanner.plan(start=start, target=end)

    # Update the progress bar
    progress["value"] = 100
    progress.update_idletasks()

    last_calculated_path = astarPlanner.finalPath
    # Draw the path on the canvas of the interface
    for i in range(len(last_calculated_path)-1):
        pt =last_calculated_path[i].tuple()
        # Draw a line between two consecutive points in the path to visualize the path
        canvas.create_line(pt[0] * section2_width / original_width, pt[1] * section2_height / original_height,
                          last_calculated_path[i+1].x * section2_width / original_width,
                            last_calculated_path[i+1].y * section2_height / original_height, fill="blue", tags="path_line", width=2)
        
    # Reset the progress bar
    progress["value"] = 0 


def on_click(canvas, original_img, section2_width, section2_height, original_width, original_height, event, progress):
        global robot_pos, end_pos

        robot_tag = "robot"   # Identify the robot position on the canvas
        arrow_tag = "arrow"  # Identify the orientation arrow of the robot on the canvas

        original_x = int(event.x * original_width / section2_width)
        original_y = int(event.y * original_height / section2_height)

        pixel_value = original_img.getpixel((original_x, original_y))
        is_white = all(value >= 245 for value in pixel_value[:3])

        if is_white:
            radius = 5  # circle radius
            if end_pos:  # If a target point already exists, remove it
                canvas.delete('end')
            end_pos = Point(original_x, original_y)
            canvas.create_oval(event.x - radius, event.y - radius, event.x + radius, event.y + radius, fill="red", tags='end')

            update_path(canvas, robot_pos, end_pos, section2_width, section2_height, original_width, original_height, original_img, progress)
            follow_path(canvas, last_calculated_path, section2_width, section2_height, original_width, original_height)
            path = get_last_calculated_path()

        else :  
            # Draw a cross to indicate that the target point cannot be placed here
            cr1 = canvas.create_line(event.x - 5, event.y - 5, event.x + 5, event.y + 5, fill="red", tags="cross", width=2)
            cr2 = canvas.create_line(event.x + 5, event.y - 5, event.x - 5, event.y + 5, fill="red", tags="cross", width=2)

            # Remove the message after 1 second
            canvas.after(1000, lambda: canvas.delete(cr1, cr2))


# Function to retrieve the last calculated path
def get_last_calculated_path():
    return last_calculated_path

# Function to move the robot using keyboard keys
def on_key_press(event, canvas, section2_width, section2_height, original_width, original_height):
    global cmd_vel_pub, stop_follow_path

    robot_tag = "robot"  # Identify the robot on the canvas
    arrow_tag = "arrow"  # Identify the arrow on the canvas
    arrow_length = 20  # Arrow length

    robot_x_avant, robot_y_avant, robot_theta_avant = get_robot_pos()

    stop_follow_path = False  

    twist_msg = Twist()

    if event.keysym == 'Up':
        twist_msg.linear.x = 0.5  # Move forward
    elif event.keysym == 'Down':
        twist_msg.linear.x = -0.5  # Move backward
    elif event.keysym == 'Left':
        twist_msg.angular.z = 0.5  # Turn left
    elif event.keysym == 'Right':
        twist_msg.angular.z = -0.5  # Turn right
        
    # Publish movement commands
    cmd_vel_pub.publish(twist_msg)

    if event.keysym == 'space':
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        cmd_vel_pub.publish(twist_msg)
        stop_follow_path = True

    # Update the robot's position on the interface
    robot_x, robot_y, robot_theta = get_robot_pos()

    canvas_x, canvas_y, robot_theta = convert_robot_pos(section2_width, section2_height, original_width, original_height)
    canvas.coords(robot_tag, canvas_x-5, canvas_y-5, canvas_x+5, canvas_y+5)
    canvas.coords(arrow_tag, canvas_x, canvas_y, canvas_x + arrow_length * math.cos(robot_theta), canvas_y + arrow_length * math.sin(robot_theta))
    canvas.update()

# Calculate commands to send to each wheel motor based on desired movement
def move_robot(linear, angular):
    global cmd_drive_pub

    if cmd_drive_pub is None:
        print("cmd_drive_pub is not initialized.")
        return 

    # Angular velocities of the wheels
    vel_left  = (2*linear - interwheel_distance*angular)/(2*left_wheel_radius)
    vel_right = (2*linear + interwheel_distance*angular)/(2*right_wheel_radius)
    
    # Send commands to the wheels via ROS topic cmd_drive
    cmd_drive_msg = Drive()
    cmd_drive_msg.drivers[0] = vel_left
    cmd_drive_msg.drivers[1] = vel_right
    cmd_drive_pub.publish(cmd_drive_msg)


async def follow_path_async(canvas, path, section2_width, section2_height, original_width, original_height):
    global last_calculated_path, end_pos, stop_follow_path

    robot_tag = "robot"  # Identify the robot on the canvas
    arrow_tag = "arrow"  # Identify the arrow on the canvas
    arrow_length = 20  # Arrow length

    robot_x, robot_y, robot_theta = get_robot_pos()

    for point in range (0, len(last_calculated_path), 2) :
        if stop_follow_path:
            print("Follow path stopped.")
            stop_follow_path = False
            canvas.delete("path_line") 
            canvas.delete("end")
            break

        ros_point_x, ros_point_y = convert_map_to_pos(last_calculated_path[point].x , last_calculated_path[point].y , section2_width, section2_height, original_width, original_height)
        dx = ros_point_x - robot_x
        dy = ros_point_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)  # Angle between the robot and the point to reach
        angle_diff = target_angle - robot_theta
        angle_diff_norm = (angle_diff % (2 * math.pi)) * 2 * math.pi
        angle_diff_f = angle_diff - angle_diff_norm
       
        # Proportional gains
        k_d_lin = 1.75
        k_d_ang = 1.5

        # First, orient the robot towards the point to reach
        while abs(angle_diff) > 0.1 and stop_follow_path == False:
            angular_speed =   max(-0.5, min(0.5, k_d_ang * angle_diff))
            move_robot(0, angular_speed) # Orient the robot towards the point to reach

            await asyncio.sleep(0.05)
            
            _, _, robot_theta = get_robot_pos()  
            
            angle_diff = target_angle - robot_theta
            angle_diff_norm = (angle_diff % (2 * math.pi)) * 2 * math.pi
            angle_diff_f = angle_diff - angle_diff_norm
      
            
        while distance > 0.4 and abs(angle_diff) <= 0.2 and stop_follow_path == False:
            linear_speed = max(0,min(k_d_lin * distance, 0.5))  # min and max to limit linear speed between 0 and 0.5 m/s
            move_robot(linear_speed, 0)

            await asyncio.sleep(0.05)
            
            robot_x, robot_y, _ = get_robot_pos()
            dx = ros_point_x - robot_x
            dy = ros_point_y - robot_y
            distance = math.sqrt(dx**2 + dy**2)

        draw_robot(canvas, robot_tag, arrow_tag, section2_width, section2_height, original_width, original_height)
        canvas_x, canvas_y, robot_theta = convert_robot_pos(section2_width, section2_height, original_width, original_height)
        canvas.coords(robot_tag, canvas_x-5, canvas_y-5, canvas_x+5, canvas_y+5)
        # Orient the robot towards the point to reach
        ang_x = canvas_x + arrow_length * math.cos(target_angle)
        ang_y = canvas_y + arrow_length * math.sin(target_angle)
        canvas.coords(arrow_tag, canvas_x, canvas_y, ang_x, ang_y)
        canvas.update()
        time.sleep(0.05) # Delay for the movement to be visible on the interface

    # When the robot reaches the target point, stop the robot
    move_robot(0, 0) # Stop the robot
    stop_follow_path = False # Reset the stop_follow_path variable
    canvas.delete("path_line")  # Remove the path line after reaching the target point
    canvas.delete("end") # Remove the target point after reaching it
    
def start_follow_path(canvas, path, section2_width, section2_height, original_width, original_height):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(follow_path_async(canvas, path, section2_width, section2_height, original_width, original_height))
    

def follow_path(canvas, path, section2_width, section2_height, original_width, original_height):
    thread_path = threading.Thread(target=start_follow_path, args=(canvas, path, section2_width, section2_height, original_width, original_height))
    thread_path.start()
        
    return thread_path
