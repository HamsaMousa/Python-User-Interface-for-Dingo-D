from os import environ
import rospy
import numpy as np
from lab_utils.plan_utils import *
import cv2 
import threading
import time
import asyncio
from PIL import Image as PilImage
from PIL import ImageDraw
from cv_bridge import CvBridge
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from path_planning import get_heading_from_quaternion, get_robot_pos, read_SIM_value

SIM = read_SIM_value()
cam_sub = None
tag_sub = None

cam_msg = Image()
def cam_callback(msg):
    global cam_msg
    cam_msg = msg

tag_array = AprilTagDetectionArray()
def tag_callback(msg):
    global tag_array
    tag_array = msg


if SIM: # Simulation
    cam_sub = rospy.Subscriber('/mobile_manip/d435i/color/image_raw', Image, cam_callback)
    tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)
else:
    cam_sub = rospy.Subscriber('/mobile_manip/d435i/color/image_raw', Image, cam_callback)
    tag_sub = rospy.Subscriber('/mobile_manip/tag_detections', AprilTagDetectionArray, tag_callback)

def resize_image(image, max_width, max_height):
    width, height = image.size
    aspect_ratio = width / height
    if width > max_width or height > max_height:
        if width > height:
            new_width = max_width
            new_height = int(max_width / aspect_ratio)
        else:
            new_height = max_height
            new_width = int(max_height * aspect_ratio)
        return image.resize((new_width, new_height)) 
    else:
        return image


def camera_view(section1_width, section1_height):
    if not cam_msg or cam_msg.height == 0 :
        print("No image received")
        return None

    else:
        bridge = CvBridge()
        # Convert the image from a ROS message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(cam_msg, desired_encoding='bgr8')

        # Convert the image from BGR to RGB
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Convert the OpenCV image to a PIL image
        image = PilImage.fromarray(cv_image_rgb)

         # Resize the image to match the section size in the GUI
        image = resize_image(image, section1_width, section1_height)

        return image

# Function to convert the tag position from the camera frame to the inertial frame
def tag_cam_to_inertial(x, y, z, orientation):
    position_tag_tag = np.array([0, 0, 0, 1])
    position_tag_cam = np.array([x, y, z])

    T_tag_cam = np.eye(4)
    T_tag_cam[:3, 3] = orientation
    T_tag_cam[:3, 3] = position_tag_cam

    if SIM : 
        position_cam_robot = np.array([0.16, 0.04, 0.463])
    else:
        position_cam_robot = np.array([0.0, 0.05, -0.137])

    T_cam_base = np.eye(4)
    rotation_cam_robot = np.array([[0, 0, 1], 
                                  [-1, 0, 0], 
                                  [0, -1, 0]])
    
    T_cam_base[:3, 3] = position_cam_robot
    T_cam_base[:3, :3] = rotation_cam_robot

    x_robot, y_robot, theta_robot = get_robot_pos()

    R_z = np.array([[np.cos(theta_robot), -np.sin(theta_robot), 0],
                    [np.sin(theta_robot), np.cos(theta_robot), 0],
                    [0, 0, 1]])
    
    T_robot_inertial = np.eye(4)
    T_robot_inertial[:3, :3] = R_z
    T_robot_inertial[:3, 3] = np.array([x_robot, y_robot, 0])

    pos_tag_cam = np.dot(T_tag_cam, position_tag_tag)
    pos_tag_robot = np.dot(T_cam_base, pos_tag_cam)
    pos_tag_inertial = np.dot(T_robot_inertial, pos_tag_robot)

    return pos_tag_inertial[0], pos_tag_inertial[1], pos_tag_inertial[2]

# Function to convert the tag position from the camera frame to the robot frame
def tag_cam_to_robot(x, y, z, orientation):
    position_tag_tag = np.array([0, 0, 0, 1])
    position_tag_cam = np.array([x, y, z])

    T_tag_cam = np.eye(4)
    T_tag_cam[:3, 3] = orientation
    T_tag_cam[:3, 3] = position_tag_cam

    if SIM : 
        position_cam_robot = np.array([0.16, 0.04, 0.463])
    else:
        position_cam_robot = np.array([0.0, 0.05, -0.137])

    T_cam_base = np.eye(4)
    rotation_cam_robot = np.array([[0, 0, 1], 
                                  [-1, 0, 0], 
                                  [0, -1, 0]])
    
    T_cam_base[:3, 3] = position_cam_robot
    T_cam_base[:3, :3] = rotation_cam_robot

    pos_tag_cam = np.dot(T_tag_cam, position_tag_tag)
    pos_tag_robot = np.dot(T_cam_base, pos_tag_cam)

    return pos_tag_robot[0], pos_tag_robot[1], pos_tag_robot[2]

# Draw a point on the top left corner of the camera view to indicate that a tag has been detected
def plotPoint(image, center, color):
    draw = ImageDraw.Draw(image)
    left_up_point = (center[0] - 5, center[1] - 5)
    right_down_point = (center[0] + 5, center[1] + 5)
    draw.rectangle([left_up_point, right_down_point], fill=color, width = 10)
    return image

def update_tag_detection(image):
    if tag_array.detections==[]:
        # print("No tag detected")
        pass 
    else:
        for detect in tag_array.detections:
            x_value = detect.pose.pose.pose.position.x
            y_value = detect.pose.pose.pose.position.y
            z_value = detect.pose.pose.pose.position.z
            orientation_value = get_heading_from_quaternion(detect.pose.pose.pose.orientation)

            x_inertial, y_inertial, z_inertial = tag_cam_to_inertial(x_value, y_value, z_value, orientation_value)
            image = plotPoint(image, (x_inertial, y_inertial), 'red')

            x_robot, y_robot, z_robot = tag_cam_to_robot(x_value, y_value, z_value, orientation_value)

# Function to check if a tag has been detected
def tag_detected():
    if tag_array.detections==[]:
        return False
    else:
        return True

# Function to print the tag information
def print_tag_info():
    if tag_array.detections==[]:
        # print("No tag detected")
        pass

    else:
        for detect in tag_array.detections:
            tag_x = detect.pose.pose.pose.position.x
            tag_y = detect.pose.pose.pose.position.y
            tag_z = detect.pose.pose.pose.position.z
            tag_orientation = get_heading_from_quaternion(detect.pose.pose.pose.orientation)

            # Check that the robot is well positioned around the tag's position
            tag_x_robot, tag_y_robot, tag_z_robot = tag_cam_to_robot(tag_x, tag_y, tag_z, tag_orientation)
            if tag_y_robot < -0.4:
                return 0  # The robot is to the left of the tag
            elif tag_y_robot > 0.4:
                return 1  # The robot is to the right of the tag

            return 2  # The robot is facing the tag

def robot_close_to_tag():
    if tag_array.detections==[]:
        # print("No tag detected")
        return False
    else : 
        for detect in tag_array.detections:
            tag_x = detect.pose.pose.pose.position.x
            tag_y = detect.pose.pose.pose.position.y
            tag_z = detect.pose.pose.pose.position.z
            tag_orientation = get_heading_from_quaternion(detect.pose.pose.pose.orientation)

            # Check that the robot is well positioned around the tag's position
            tag_x_robot, tag_y_robot, tag_z_robot = tag_cam_to_robot(tag_x, tag_y, tag_z, tag_orientation)
            distance = math.sqrt((tag_x_robot - 0)**2 + (tag_y_robot - 0)**2)  # The robot is at 0,0 in its own frame, and tag_x_robot, tag_y_robot are in the robot's frame

            if distance <= 1 and -0.4 <= tag_y_robot <= 0.4 :
                return True

        return False
    

# Function to add detected tags to the map in the GUI 
async def add_tag_to_map_async(canvas, original_img, section2_width, section2_height, original_width, original_height):
    map_res = 0.02
    detected_tags = set()
    tag_canvas_x_list = set()
    tag_canvas_y_list = set()
    tag_canvas_x_precedent = 0
    tag_canvas_y_precedent = 0

    while True : 
        if tag_array.detections:
            for detection in tag_array.detections:
                tag_id = detection.id[0]
                if tag_id not in detected_tags:
                    tag_x, tag_y, _ = tag_cam_to_inertial(detection.pose.pose.pose.position.x,
                                                        detection.pose.pose.pose.position.y,
                                                        detection.pose.pose.pose.position.z,
                                                        get_heading_from_quaternion(detection.pose.pose.pose.orientation))

                    if SIM:
                        offset_x = 0.0
                        offset_y = 0.0
                    else:
                        offset_x = 4.6 
                        offset_y = -4.2

                    x_pos = 23.7 + offset_x
                    y_pos = 22.6 + offset_y

                    tag_x = tag_x + x_pos
                    tag_y = tag_y + y_pos

                    tag_canvas_x = (tag_x / map_res) * (section2_width / original_width)
                    tag_canvas_y = (tag_y / map_res) * (section2_height / original_height)

                    # Add the tag to the map in the GUI by drawing a blue point
                    canvas.create_oval(tag_canvas_x - 5, tag_canvas_y - 5, tag_canvas_x + 5, tag_canvas_y + 5, fill='blue', tags="tag")
                    # Add the tag number above the drawn point corresponding to the tag position
                    canvas.create_text(tag_canvas_x, tag_canvas_y - 14, text=tag_id, fill='blue', font=('Arial', 10))

                    detected_tags.add(tag_id)

                else : 
                    continue

        await asyncio.sleep(0.1)
        

def start_add_tag_to_map(canvas, original_img, section2_width, section2_height, original_width, original_height):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(add_tag_to_map_async(canvas, original_img, section2_width, section2_height, original_width, original_height))

def add_tag_to_map(canvas, original_img, section2_width, section2_height, original_width, original_height):
    thread_tag = threading.Thread(target=start_add_tag_to_map, args=(canvas, original_img, section2_width, section2_height, original_width, original_height))
    thread_tag.start()

    return thread_tag
