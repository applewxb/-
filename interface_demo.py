#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Bool
import sys
rospy.timer.TimerEvent

from offboard_pkg.msg import Line
from offboard_pkg.msg import Lines
from offboard_pkg.msg import Distance

import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco

'''
=========================================================================
set point to fly
=========================================================================
'''
current_state = State()
cur_position = [0, 0, 0]
goal_position = [0,0,0]
path = [[0,0],[-1.7,0],[-1.7,2.87],[0,3.43],[0,5.37],[-1.9,5.37],[-1.9,7],[0,7.6]] 
aruco_pos_for_mavros = [0,0,0]
idx = 0
land_flag = 0
flag_pose_pub = False

aruco_pos = [0, 0, 0]
bridge = CvBridge()
last_seen_time = None

def state_cb(msg):
    global current_state
    current_state = msg
    
def pos_cb(msg):
    global cur_position
    cur_position[0]=msg.pose.position.x;
    cur_position[1]=msg.pose.position.y;
    cur_position[2]=msg.pose.position.z;

def SetPoint(event):
    global local_pos_pub
    global offboard_flag
    global pose
    global idx
    global path
    global last_req
    global flag_pose_pub
    global aruco_pos_for_mavros
    global land_flag
    global last_seen_time
    # if(not current_state.connected):
    #     return
    # if (rospy.is_shutdown):
    #    sys.exit(0)     
    
    if((abs(cur_position[0])<0.1)and(abs(cur_position[1])<0.1)and(abs(cur_position[2]-HEIGHT)<0.1)and((rospy.Time.now() - last_req)>rospy.Duration(5.0))and idx < len(path)):
             pose.pose.position.x = path[idx][0]
             pose.pose.position.y = path[idx][1]
             pose.pose.position.z = HEIGHT
             pose.pose.orientation.z = 0.707 
             pose.pose.orientation.w = 0.707
             last_req = rospy.Time.now()
             idx += 1

    if((abs(cur_position[0]-pose.pose.position.x)<0.1) and (abs(cur_position[1] - pose.pose.position.y)<0.1) and (abs(cur_position[2]-HEIGHT)<0.1) and idx < len(path)):
        print(idx,len(path))
        pose.pose.position.x = path[idx][0]
        pose.pose.position.y = path[idx][1]
        pose.pose.position.z = HEIGHT
        aruco_pos_for_mavros[0] = path[idx][0]
        aruco_pos_for_mavros[1] = path[idx][1]
        aruco_pos_for_mavros[2] = 1.5
        idx += 1
        pass
    elif((abs(cur_position[0]-pose.pose.position.x)<0.1) and (abs(cur_position[1] - pose.pose.position.y)<0.1) and (abs(cur_position[2]-HEIGHT)<0.1) or land_flag):
         # set_mode_client.call(land_set_mode)
        # if(set_mode_client.call(land_set_mode).mode_sent == True):
            # rospy.loginfo("land enabled")
        # offboard_flag = 1
        land_flag = True
        timeout_duration = rospy.Duration(15)
        if aruco_pos:
	    last_seen_time = rospy.Time.now()
	    current_x = pose.pose.position.x
	    current_y = pose.pose.position.y
	    current_z = pose.pose.position.z
	   
	   
	    specified_position = (2, 8.5, 0)

            pose.pose.position.x = specified_position[0]
            pose.pose.position.y = specified_position[1]
            pose.pose.position.z = 1

            local_pos_pub.publish(pose)

          
        elif last_seen_time is not None and rospy.Time.now() - last_seen_time > timeout_duration:
	    time.sleep(2)
            set_mode_client.call(land_set_mode)
            if(set_mode_client.call(land_set_mode).mode_sent == True):
                rospy.loginfo("land enabled")
            offboard_flag = 1


          
        
    flag_pose_pub = True
    local_pos_pub.publish(pose)


def image_callback(msg):
    global aruco_pos
    
    #print("Encoding:", msg.encoding)
    #print("Timestamp:", msg.header.stamp)
    #print("Size:", len(msg.data))    
   
    np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

    
    cv_image = cv2.resize(np_arr, (480, 480), interpolation=cv2.INTER_AREA)

   
    if cv_image is None or cv_image.size == 0:
        print("Image is empty")
    else:
        # print("Image shape:", cv_image.shape)
        # cv2.imwrite("/home/rflysim/Desktop/competition_auto_aruco/competition_ws/image.jpg", cv_image)
        pass
    
    aruco_pos = detect_aruco(cv_image)


def detect_aruco(img):

    if img is None or img.size == 0:
        
        return [0, 1, 0]

   
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    
   
    corners, ids, _ = aruco.detectMarkers(img, aruco_dict, parameters=parameters)
    
    
    if ids is not None and len(ids) > 0:
        
        corners_first_marker = corners[0]
        
        
        center_x = np.mean(corners_first_marker[0][:, 0])
        center_y = np.mean(corners_first_marker[0][:, 1])
        
        
        img_center_x = img.shape[1] / 2
        img_center_y = img.shape[0] / 2
        
        cv2.circle(img, (int(center_x), int(center_y)), 5, (255, 0, 0), -1) 
        cv2.circle(img, (int(img_center_x), int(img_center_y)), 5, (0, 0, 255), -1)   
        cv2.imwrite("/home/rflysim/Desktop/competition_auto_aruco/competition_ws/image.jpg", img)

	cv2.imshow("show",img)
	cv2.waitKey(1)

        offset_x = center_x - img_center_x
        offset_y = center_y - img_center_y
        
       
        
        drone_height = 1.1  
        fov_x = 80 
        fov_y = 60  
        
        
        distance_x = offset_x * drone_height / np.tan(np.radians(fov_x / 2))
        distance_y = offset_y * drone_height / np.tan(np.radians(fov_y / 2))
        print("==========================",distance_x, distance_y, drone_height)
        
        return [distance_x, distance_y, drone_height]
    else:
        
        return None


'''
=========================================================================
use distance of drone's front,bihand,left,right 
=========================================================================
'''
def dist_cb(dists):

    pass

def lines_cb(lines):
    pass

def aruco_pose_cb(msg):

    global aruco_pos_for_mavros

    print("****",msg)
    aruco_pos_for_mavros[0] = msg.position.x
    aruco_pos_for_mavros[1] = msg.position.y
    aruco_pos_for_mavros[2] = msg.position.z
    pass

if __name__ == "__main__":
    rospy.init_node("interface_demo")
    HEIGHT = 1
    #rospy.get_param('~height')
    AUTO_ARM_OFFBOARD = rospy.get_param('~auto_arm_offboard')
    pose = PoseStamped()

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, callback = pos_cb)

    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    aruco_pos_sub = rospy.Subscriber("/aruco_pose_mavros",PositionTarget,callback=aruco_pose_cb)
    image_sub = rospy.Subscriber("/rflysim/sensor1/img_rgb", Image,callback=image_callback)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    dis_sub = rospy.Subscriber("/objs_dist",Distance,callback=dist_cb)
    
    lines_sub = rospy.Subscriber("/objs_line",Lines,callback=lines_cb)
    
    rospy.Timer(rospy.Duration(0.1),callback= SetPoint)


    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    land_set_mode = SetModeRequest()
    land_set_mode.custom_mode = 'AUTO.LAND'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    offboard_flag = 0

    rate = rospy.Rate(20)
    
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()


    pose.pose.position.x = 0
    pose.pose.position.y = 0
    #pose.pose.position.z = 0.5
    pose.pose.position.z = HEIGHT
    pose.pose.orientation.z = 0.707
    pose.pose.orientation.w = 0.707
    
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()
    
    last_req = rospy.Time.now()
    while(not rospy.is_shutdown()):
        if(AUTO_ARM_OFFBOARD == True):
            if(current_state.mode != "OFFBOARD" and flag_pose_pub and (rospy.Time.now() - last_req) > rospy.Duration(5.0) and (offboard_flag == 0)):
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0) and (offboard_flag == 0)):
                    if(arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()
        rate.sleep()

    cv2.destroyAllwindows()

