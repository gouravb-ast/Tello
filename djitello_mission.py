from djitellopy import Tello
import time
import cv2 as cv
from cv2 import aruco
import numpy as np
import itertools
import math


#drone conenction and stream on
tello = Tello()
tello.connect()
tello.streamon()

#---<ArUCO and camera Part>---#
#aruco
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
marker_param = aruco.DetectorParameters_create()
marker_size= 5.5 #cm
#camera calibration files
calib_path="./cal_images_tello/"
camera_matrix = np.loadtxt(calib_path+"cameraMatrix.txt", delimiter=',')
camera_distortion = np.loadtxt(calib_path+"cameraDistortion.txt", delimiter=',')
#180 degree rotation matrix around the x axis
R_flip = np.zeros((3,3),dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] = -1.0
R_flip[2,2] = -1.0
#---</ArUCO and camera Part>---#

#define marker ids as per levels for drone actions
level_one_markers = [0,1,4]
level_two_markers = [5,6,7,8]
level_three_markers = [10,11,12,13]
move_up_markers = [4,9]
land_markers = [3]

def drone_takeoff():
    try:
        if key == ord('t') or key == ord('T'):
            tello.takeoff()
            print("Drone takeoff successful !")
            #tello.move_down(15)
    except:
        print("takeoff command failed !")

def drone_land():
    try:
        if key == ord('l') or key == ord('L'):
            tello.land()
            print("Drone land successful !")
    except:
        print("land command failed !")


var1 = 0

print("var1 set to", var1)

while True:

    key = cv.waitKey(1) & 0xff

    #drone telemetry
    drone_battery = tello.get_battery()
    drone_height = tello.get_height()

    #stream
    frame = tello.get_frame_read().frame
    
    cv.putText(frame,f"Drone Battery: {drone_battery} %",(20,40),cv.FONT_HERSHEY_PLAIN,1.3,(0,255,0),1,cv.LINE_AA)
    cv.putText(frame,f"Drone Height: {drone_height} cm",(20,60),cv.FONT_HERSHEY_PLAIN,1.3,(0,255,0),1,cv.LINE_AA)
    
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject  = aruco.detectMarkers(gray_frame, marker_dict, parameters = marker_param) #cameraMatrix=camera_matrix, distCoeff=camera_distortion 

    #position of markers
        #-- ret = [rvec, tvec. ?]
        #-- array of rotation and position of each marker in camera frame
        #-- rvec = [[rvec_1],[rvec_2],...] attitude of the marker respect to camera frame
        #-- tvec = [[tvec_1],[tvec_2],..] position of the marker in the camera frame
    
    try:
        ret = aruco.estimatePoseSingleMarkers(marker_corners, marker_size, camera_matrix, camera_distortion)    
        
        #unpack the output ret and get only the first
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
        print(rvec)   
        #draw the detected marker and put a reference frame over it
        # aruco.drawDetectedMarkers(frame, marker_corners)
        cv.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 10, 2)
        print("axis")
    except:
        pass

    for id, corners in zip(itertools.repeat(marker_IDs), marker_corners):
        cv.polylines(frame, [corners.astype(np.int32)], True, (0,0,255),4,cv.LINE_AA)

        corners = corners.reshape(4,2)
        corners= corners.astype(int)
        id = id.astype(int)

        top_left = corners[0].ravel()    
        top_right = corners[1].ravel()
        bottom_right = corners[2].ravel()
        bottom_left = corners[3].ravel()

        pixel_dist_h = math.sqrt(((top_right[0]-top_left[0])**2)+((top_right[1]-top_left[1])**2))
        pixel_dist_v = math.sqrt(((bottom_left[0]-top_left[0])**2)+((bottom_left[1]-top_left[1])**2))

        pdhi = int(pixel_dist_h/2) #pixel distance horizontal integer 
        pdvi = int(pixel_dist_v/2) #pixel distance vertical integer
        
        tlxi = int(top_left[0])
        tlyi = int(top_left[1])
        cx =  (pdhi + tlxi) 
        cy =  (pdvi + tlyi)     
        # for sample_actual_dist of 30cm the sample_pixel_dist is (174)
        # act_dist and pixel_dist are inversly propotional so, act_dist x= 1/ pixel_dist
        # constant = sample_act_dist x sample_pixel_dist
        # so for 30cm = 1 / 174 = 0.006
        # i.e. constant = 0.006
        actual_dist_h = (1/ pixel_dist_h)*5220 #cm

        cv.putText(frame,f"id:{id[0]}",top_left,cv.FONT_HERSHEY_PLAIN,1.3,(0,0,0),2,cv.LINE_AA)
        cv.putText(frame,f"distance:{actual_dist_h}",(20,20),cv.FONT_HERSHEY_PLAIN,1.3,(0,255,0),1,cv.LINE_AA)



    #velocity commands by positioning and keyboard control
    lr,fb,ud,yaw = 0,0,0,0
    
    speed =100
    if key == ord ('w'):
        fb = speed
    elif key == ord ('s'):
        fb = -speed
    elif key == ord ('d'):
        lr = speed
    elif key == ord ('a'):
        lr = -speed
    elif key == ord ('r'):
        ud = speed
    elif key == ord ('f'):
        ud = -speed
    elif key == ord ('c'):
        yaw = speed
    elif key == ord ('z'):
        yaw = -speed

    tello.send_rc_control(lr,fb,ud,yaw)   #left-right, forward-backward, up-down, yaw

    # #waypoint movement
            # print("waypoint flag in script", waypoint_flag)
            # print(id)
            # var = id[0][0]   
            # if waypoint_flag:    
            #     print("waypoint action")       
            #     try:
            #         if var == var1:
            #             if var in level_one_markers:
            #                 print("l1",id)
            #                 tello.move_right(60)
            #                 var1 += 1
            #                 print("Level one marker",var1)
            #                 print("moving Right!")
            #                 waypoint_flag = False
                        
                        
            #             elif var in land_markers:
            #                 print("land_markers",id)
            #                 tello.land()
            #                 var1 += 1   
               
            #     except:
            #         print("failed to pass position setpoint !")
    #Display drone feed
    cv.imshow("Drone Feed", frame)

    drone_takeoff()
    
    drone_land()
    
    #exit from code    
    if key == ord("q"): # ESC
        tello.streamoff()
        break