#!/usr/bin/python3

#from curses.panel import bottom_panel
import cv2 as cv
from cv2 import aruco
import numpy as np
import itertools
import math
import time
from djitellopy import Tello
import time
import json
import websocket


#drone
#Tello 
tello = Tello()
tello.connect()
tello.streamon()
frame_read = tello.get_frame_read()


def tello_takeoff():
    try:
        if key == ord('t') or key == ord('T'):
            tello.takeoff()
            print("Drone takeoff successful !")
            #tello.move_down(15)
    except:
        print("takeoff command failed !")
    
def tello_land():
    try:
        if key == ord('l') or key == ord('L'):
            tello.send_rc_control(0,0,0,0)
            time.sleep(0.5)
            tello.land()
            print("Drone land successful !")
    except:
        print("land command failed !")

#ArUCO
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
param_marker = aruco.DetectorParameters_create()
marker_size= 5.5 #cm
#camera calibration files
calib_path="./tello_camera_calib_files/"
camera_matrix = np.loadtxt(calib_path+"cameraMatrix.txt", delimiter=',')
camera_distortion = np.loadtxt(calib_path+"cameraDistortion.txt", delimiter=',')
#180 degree rotation matrix around the x axis
R_flip = np.zeros((3,3),dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] = -1.0
R_flip[2,2] = -1.0
#check if matrix is a valid rotation matrix
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

def text_box(frame):
    # creating text area
    x, y, w, h = 5, 5, 300, 50
    sub_img = frame[y:y+h, x:x+w]
    white_rect = np.ones(sub_img.shape, dtype=np.uint8) * 255
    res = cv.addWeighted(sub_img, 0.1, white_rect, 0.5, 1.0)
    frame[y:y+h, x:x+w] = res

#<------------------------------------------------------------------------------>#
#position correction without yaw 
# pDistError = 0
# def pos_correction(x_dist, dist, y_dist, pDistError):
    
#     lr_s, fb_s, ud_s, yaw_s = 0, 0, 0, 0

#     #dist correction
#     kp_d=1.5
#     kd_d=0.9
#     if dist !=0:
#         dist_error =  int(dist - 50)
#         fb_s = (kp_d * dist_error) + (kd_d * (dist_error - pDistError))
#         fb_s = int(np.clip(fb_s,-100,100))
#         print("previousError:", pDistError)
#     else:
#         dist_error = 0
#     #---#
#     # if dist < 47 and dist > 0 :
#     #     dist_error =  dist - 53
#     #     fb_s = int(kp_d * dist_error)
#     # elif dist > 53:
#     #     dist_error =  dist - 47
#     #     fb_s = int(kp_d * dist_error)
#     # else:
#     #     fb_s = 0
#     #---#

#     #up-down correction
#     #---#
#     ud_error =abs(y_dist)
#     kp_ud = 0.99 
#     if y_dist < -3:
#         ud_s = int(-(kp_ud * ud_error))
#     elif y_dist >3:
#         ud_s = int((kp_ud * ud_error))
#     #---#

#     #left-right correction
#     #---#
#     lr_error = abs(x_dist)
#     kp_lr = 0.99
#     if x_dist < -3:        
#         lr_S = kp_lr * lr_error
#     elif x_dist >3:
#         lr_s = kp_lr * lr_error
#     #---#

#     return lr_s, fb_s, ud_s, yaw_s, dist_error

#<------------------------------------------------------------------------------>#

#-------POSITION CORRECTION WITH YAW---------# 
p_lr_error = 0
p_fb_error = 0
p_ud_error = 0
p_yaw_error = 0
currentTime = time.time()
previousTime = currentTime
cI_ud = 0
cI_fb = 0

def pos_correction_withYaw(frame_centre, marker_centre_x, marker_centre_y, fb_pos, yaw_pos, p_lr_error, p_fb_error, p_ud_error, p_yaw_error, currentTime, previousTime, cI_ud, cI_fb):
    
    lr_s, fb_s, ud_s, yaw_s = 0, 0, 0, 0
    frame_centre_x , frame_centre_y = frame_centre[0], frame_centre[1]
    currentTime = time.time()
    deltaTime = currentTime - previousTime

    #yaw correction
    kp_yaw = 0.15
    kd_yaw = 0.25
    if marker_centre_x !=0:
        yaw_error = marker_centre_x - frame_centre_x 
        yaw_s = kp_yaw * yaw_error + kd_yaw * (yaw_error - p_yaw_error)
        yaw_s = int(np.clip(yaw_s,-100,100))
    else:
        yaw_error = 0

    #up-down correction
    kp_ud = 0.5
    kd_ud = 0.0025
    ki_ud = 0
    if marker_centre_y !=0 and (frame_centre_y+30 < marker_centre_y or marker_centre_y< frame_centre_y-30) :
        ud_error = frame_centre_y - marker_centre_y
        cI_ud += ud_error * deltaTime
        ud_delta_error = ud_error - p_ud_error
        ud_s = (kp_ud * ud_error) + (ki_ud * cI_ud) + (kd_ud * (ud_delta_error / deltaTime)) if deltaTime>0 else 0   
        ud_s = int(np.clip(ud_s, -60, 60))
        print("ud_D:", kd_ud * (ud_delta_error / deltaTime))
    else:
        ud_error = 0

    #forward-backward correction
    kp_fb = 1.5
    kd_fb = 0.009
    ki_fb = 1
    if fb_pos !=0 : #and (52<fb_pos or fb_pos<48):
        fb_error =  int(fb_pos - 50)
        cI_fb += fb_error * deltaTime
        fb_delta_error = fb_error - p_fb_error
        fb_s = (kp_fb * fb_error) + (ki_fb * cI_fb) +(kd_fb * (fb_delta_error / deltaTime)) if deltaTime>0 else 0
        fb_s = int(np.clip(fb_s,-100,60))
    else:
        fb_error = 0

    #orientation correction with left-right
    yaw_ab_pos= abs(yaw_pos)
    if 0<=yaw_ab_pos<=5:
        yaw_ab_pos = 0
    elif 6<=yaw_ab_pos<=8:
        yaw_ab_pos = 7
    elif 9<=yaw_ab_pos<=11:
        yaw_ab_pos = 10
    elif 12<=yaw_ab_pos<=14:
        yaw_ab_pos = 13
    elif 15<=yaw_ab_pos<=17:
        yaw_ab_pos = 16
    elif 18<=yaw_ab_pos<=20:
        yaw_ab_pos = 19
    elif 21<=yaw_ab_pos<=23:
        yaw_ab_pos = 22
    elif 24<=yaw_ab_pos<=26:
        yaw_ab_pos = 25
    elif 27<=yaw_ab_pos<=29:
        yaw_ab_pos = 28
    elif 30<=yaw_ab_pos<=32:
        yaw_ab_pos = 31
    elif 33<=yaw_ab_pos<=35:
        yaw_ab_pos = 34
    else:
        yaw_ab_pos= abs(yaw_pos)

    if 40<= fb_pos <=65 and 450<= marker_centre_x <=510 and yaw_ab_pos >0:
        kp_lr = 1
        kd_lr = 0.1
        lr_error = yaw_pos
        lr_s = kp_lr * yaw_pos + kd_lr * (lr_error - p_lr_error)    
        lr_s = int(np.clip(lr_s, -100, 100))
    else:
        lr_error = 0

    return lr_s, fb_s, ud_s, yaw_s, lr_error, fb_error, ud_error, yaw_error, currentTime

#---------------------------------#

#------WAYPOINT MARKERS------#
l1_move_right = [0,1,2,8]
l1_move_up = [3]
l2_move_left = [4,5,6]
l2_move_down = [7]
#----------------------------#

#----------WEBSOCKET-------------#
try:
    ws = websocket.WebSocket()
    ws.connect("ws://127.0.0.1:8000/ws/tello/")  
except:
    pass
#--------------------------------#

#INITIAL POINTER SET
var1 = 0

while True:
    drone_battery = tello.get_battery()
    frame = frame_read.frame
    text_box(frame)
    f_shape = frame.shape
    frame_centre = (f_shape[1]//2 , f_shape[0]//2) 

    key = cv.waitKey(1) & 0xff
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_frame, marker_dict, parameters = param_marker)

    #position of markers
        #-- ret = [rvec, tvec. ?]
        #-- array of rotation and position of each marker in camera frame
        #-- rvec = [[rvec_1],[rvec_2],...] attitude of the marker respect to camera frame
        #-- tvec = [[tvec_1],[tvec_2],..] position of the marker in the camera frame
    
    lr_pos = 0
    ud_pos = 0
    fb_pos = 0
    yaw_pos = 0
    yaw_ab_pos = 0
    marker_centre_x = 0
    marker_centre_y = 0
    
    try:
        
        ret = aruco.estimatePoseSingleMarkers(marker_corners, marker_size, camera_matrix, camera_distortion)    
        
        #unpack the output ret and get only the first
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]        
    
        #draw the detected marker and put a reference frame over it
        # aruco.drawDetectedMarkers(frame, marker_corners)
        cv.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 10, 2)
        
        #position in camera frame
        lr_pos = int(tvec[0]) * -1
        ud_pos = int(tvec[1]) * -1
        fb_pos = int(tvec[2]) 
        marker_position = f"Marker Position x={lr_pos} y={ud_pos} z={fb_pos}"
        cv.putText(frame,marker_position,(10,100),cv.FONT_HERSHEY_PLAIN,1.3,(0,0,255),1,cv.LINE_AA)
        cv.putText(frame,f"Distance:{fb_pos}",(10,25),cv.FONT_HERSHEY_PLAIN,1.3,(255,255,255),1,cv.LINE_AA)


        #obtaining Rotation matrix -> tag to camera
        R_ct = np.matrix(cv.Rodrigues(rvec)[0])
        R_ct = R_ct.T

        #getting attitude in terms of euler 321 (needs to be flipped first)
        pitch_marker,yaw_marker,roll_marker = rotationMatrixToEulerAngles(R_flip*R_ct)
        
        #center of frame
        cv.circle(frame, (frame_centre[0], frame_centre[1]), 5, (0,0,255), -1)
        
        marker_attitude = f"roll:{int(math.degrees(roll_marker))} pitch:{int(math.degrees(pitch_marker))} yaw:{int(math.degrees(yaw_marker))}"
        cv.putText(frame,marker_attitude,(10,150),cv.FONT_HERSHEY_PLAIN,1.3,(255,0,0),1,cv.LINE_AA)
        
        #marker's attitude
        yaw_pos = int(math.degrees(yaw_marker))
        
    except:
        pass

    #centre target area
    sq_cordinates = np.array([[[frame_centre[0]-40, frame_centre[1]-40],[frame_centre[0]+40, frame_centre[1]-40], [frame_centre[0]+40, frame_centre[1]+40], [frame_centre[0]-40, frame_centre[1]+40]]],np.int32) 
    cv.polylines(frame, sq_cordinates, True, (0,0,255), 2)        
    
    
    for ids, corners in zip(itertools.repeat(marker_IDs), marker_corners):
        
        cv.polylines(frame, [corners.astype(np.int32)], True,(0,0,255),4,cv.LINE_AA)
        corners = corners.reshape(4,2)
        corners = corners.astype(int)
        

        top_left = corners[0].ravel()    
        top_right = corners[1].ravel()
        bottom_right = corners[2].ravel()
        bottom_left = corners[3].ravel()

        x_sum = corners[0][0].ravel()+ corners[1][0].ravel()+ corners[2][0].ravel()+ corners[3][0].ravel()
        y_sum = corners[0][1].ravel()+ corners[1][1].ravel()+ corners[2][1].ravel()+ corners[3][1].ravel()
             
        marker_centre_x = int(x_sum*.25)
        marker_centre_y = int(y_sum*.25)
        cv.circle(frame, (marker_centre_x,marker_centre_y),5,(0,0,255),-1)
        cv.putText(frame,f"{marker_centre_x,marker_centre_y}",(marker_centre_x,marker_centre_y),cv.FONT_HERSHEY_PLAIN,1.3,(255,255,255),1,cv.LINE_AA)
        cv.putText(frame,f"id:{ids[0]}",top_left,cv.FONT_HERSHEY_PLAIN,1.3,(0,255,0),2,cv.LINE_AA)

        #-------WAYPOINT TRIGGER---------#
        var = ids[0][0]
        print("var",var)

        try:

            if 0<=abs(yaw_pos)<=3 and frame_centre[0]-30 <= marker_centre_x <= frame_centre[0]+30 and 45<= fb_pos <=55 and frame_centre[1]-50 <= marker_centre_y <= frame_centre[1]+50:
                tello.send_rc_control(0,0,0,0)
                time.sleep(0.1)
            
                if var == var1:
                    if var in l1_move_right:
                        tello.move_right(60)
                        var1 += 1
                        print("Tello Moving Right!")
                        print("var1",var1)
                    
                    
                    elif var in l1_move_up:
                        tello.move_up(60)
                        var1 += 1
                        print("Tello Moving Up!")
                        print("var1",var1)

                    elif var in l2_move_left:
                        tello.move_left(60)
                        var1 +=1
                        print("Tello Moving Left")
                        print("var1",var1)

                    elif var in l2_move_down:
                        tello.move_down(60)
                        var1 == 0
                        print("Tello Moving Left")
                        print("var1",var1)
        except:
            pass

        #--------------------------------#
    
    #-----------VELOCITY SETPOINT COMMAND-------------#
    #velocity commands by positioning and keyboard control
    #lr,fb,ud,yaw = 0,0,0,0
    #lr,fb,ud,yaw, pDistError = pos_correction(lr_pos,fb_pos, ud_pos, pDistError)  
    lr,fb,ud,yaw, p_lr_error, p_fb_error, p_ud_error, p_yaw_error, previousTime = pos_correction_withYaw(frame_centre, marker_centre_x, marker_centre_y, fb_pos, yaw_pos, p_lr_error, p_fb_error, p_ud_error, p_yaw_error, currentTime, previousTime, cI_ud, cI_fb)
    
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

    #----------------------------------------------#
    
    #------------POSITION SET COMMAND USING KEYS------------#
    if key == ord ('y'):
        tello.send_rc_control(0,0,0,0)
        time.sleep(0.1)
        tello.move_left(60)
    elif key == ord ('i'):
        tello.send_rc_control(0,0,0,0)
        time.sleep(0.1)
        tello.move_right(60)
    elif key == ord('j'):
        tello.send_rc_control(0,0,0,0)
        time.sleep(0.1)
        tello.move_down(60)
    elif key == ord('u'):
        tello.send_rc_control(0,0,0,0)
        time.sleep(0.1)
        tello.move_up(60)
    #-------------------------------------------------------#

    #---------------------QR CODE READER--------------------#

    #-------------------------------------------------------#
    
    #-------------------WEBSOCKET DATA PUSH-----------------#

    #-------------------------------------------------------#
    cv.putText(frame,f"Drone Battery:{drone_battery}",(10,45),cv.FONT_HERSHEY_PLAIN,1.3,(255,255,255),1,cv.LINE_AA)

    cv.imshow("frame", frame)

    tello_takeoff()
    tello_land()
    
    if key == ord('q'):
        tello.streamoff()
        break


cv.destroyAllWindows()