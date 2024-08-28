from mobile_manip.srv import ReachName, GetValues, ReachValues
from control_msgs.msg import FollowJointTrajectoryActionFeedback
from apriltag_ros.msg import AprilTagDetectionArray
import rospy
import math
from os import environ
from path_planning import get_heading_from_quaternion, read_SIM_value
from camera import tag_cam_to_robot

SIM = read_SIM_value()
tag_sub = None
arm_trajectory_sub = None
arm_goal_sub = None

tag_x = 0
tag_y = 0
tag_z = 0
tag_orientation = []
tag_array = AprilTagDetectionArray()

def tag_callback(msg):
    global tag_array, tag_x, tag_y, tag_z, tag_orientation
    tag_array = msg

arm_msg = FollowJointTrajectoryActionFeedback()
def feedback_callback(msg):
    global arm_msg
    arm_msg = msg

if SIM: # Simulation
    tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_callback)
    arm_traject_sub = rospy.Subscriber("/mobile_manip/arm_gen3_lite_joint_trajectory_controller/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback, feedback_callback)
else: # Real robot
    tag_sub = rospy.Subscriber('/mobile_manip/tag_detections', AprilTagDetectionArray, tag_callback)
    arm_traject_sub = rospy.Subscriber("/mobile_manip/arm_gen3_lite_joint_trajectory_controller/follow_joint_trajectory/feedback", FollowJointTrajectoryActionFeedback, feedback_callback)


def reach_coords():
    x = 0.5
    y = 0.0
    z = 0.6
    desired_pos = [x, y, z, -90, 0, -90, 0] # x, y, z, roll, pitch, yaw, gripper 
    rospy.wait_for_service('/mobile_manip/reach_cartesian')
    reach_pose = rospy.ServiceProxy('/mobile_manip/reach_cartesian', ReachValues)

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
            distance = math.sqrt((tag_x_robot - 0)**2 + (tag_y_robot - 0)**2) # The robot is at 0,0 in its own frame, and tag_x_robot, tag_y_robot are in the robot's frame
            print("Distance du tag au robot: ", distance)
            
            if distance <= 1 and -0.4 <= tag_y_robot <= 0.4 :  # The y value is used to check the robot's offset to the right or left of the tag to better center the robot in front of the tag
                reach_pose(desired_pos)
                print("Desired positions : ", desired_pos)
            else:
                print("Le robot n'est pas assez proche du tag")


def reach_recorded_home(): 
    actual_positions = arm_msg.feedback.actual.positions
    print("Actual positions home : ", actual_positions)

    rospy.wait_for_service('/mobile_manip/reach_name')
    reach_recorded_pose = rospy.ServiceProxy('/mobile_manip/reach_name', ReachName)
    reach_recorded_pose('home')

    desired_positions = arm_msg.feedback.desired.positions
    print("Desired positions home : ", desired_positions)

def reach_recorded_vertical():
    actual_positions = arm_msg.feedback.actual.positions
    print("Actual positions vertical : ", actual_positions)

    rospy.wait_for_service('/mobile_manip/reach_name')
    reach_recorded_pose = rospy.ServiceProxy('/mobile_manip/reach_name', ReachName)
    reach_recorded_pose('vertical')

    desired_positions = arm_msg.feedback.desired.positions
    print("Desired positions vertical : ", desired_positions)

def reach_recorded_retract():
    actual_positions = arm_msg.feedback.actual.positions
    print("Actual positions retract : ", actual_positions)

    rospy.wait_for_service('/mobile_manip/reach_name')
    reach_recorded_pose = rospy.ServiceProxy('/mobile_manip/reach_name', ReachName)
    reach_recorded_pose('retract')

    desired_positions = arm_msg.feedback.desired.positions
    print("Desired positions retract : ", desired_positions)




    

