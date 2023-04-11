
from djitellopy import Tello
import time
import cv2
from cv2 import aruco
import numpy as np
import itertools
import math

#drone 
tello = Tello()
tello.connect()
tello.streamon()

#aruco
marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
marker_param = aruco.DetectorParameters_create()

level_one_markers = [0,1,4]
level_two_markers = [5,6,7,8]
level_three_markers = [10,11,12,13]
move_up_markers = [4,9]
land_markers = [3]

def drone_takeoff():
    try:

        if key == ord('t') or key == ord('T'):
            tello.takeoff()
            time.sleep(2)
            #tello.move_down(15)
    except:
        print("takeoff command failed !")

def drone_land():
    try:

        if key == ord('l') or key == ord('L'):
            tello.land()

    except:
        print("land command failed !")


#drone positioning
#     pError_y = 0
def drone_positioning():   
    #lr, fb, ud, yaw, waypoint_flag = 0,0,0,0,False       
    # left-right adjustment
    # try:
    #     image_width =960 #need to be defined

    #     error_y = cx - image_width //2
    #     PID = [0.4,0.4,0]
    
    #     yaw = PID[0]*error_y+PID[1]*(error_y - pError_y)
    #     yaw = int(np.clip(yaw,-100,100))
    #     pError_y = error_y

    # except:
    #     print("Failed to adfjust yaw !")
    
    # try:
    #     image_width =960 #need to be defined

    #     error_y = cx - image_width //2
    #     PID = [0.4,0.4,0]
    #     pError_y = 0
    #     lr = PID[0]*error_y+PID[1]*(error_y - pError_y)
    #     lr = int(np.clip(yaw,-100,100))
    #     pError_y = error_y

    # except:
    #     print("Failed to adfjust yaw !")    

    #up-down adjustment using PID
    # try:

    #     image_height =720 #need to be defined
    #     error_ud = cy - image_height //2
    #     PID = [0.4,0.4,0]
    #     pError_ud = 0
    #     ud = PID[0]*error_ud+PID[1]*(error_ud - pError_ud)
    #     ud = int(np.clip(ud, -100,100))
    #     pError_ud = error_ud

    # except:
    #     print("Failed to adfjust up-down !")



    fb_range = [45,55] #distance from ArUCO marker in cm
    
    if actual_dist_h >= fb_range[0] and actual_dist_h <= fb_range[1]:
        fb = 0    
        print("distance good",actual_dist_h)
        waypoint_flag = True           
        print("waypoint_flag set to :", waypoint_flag)

    elif actual_dist_h < fb_range[0]:
        fb = -30
        waypoint_flag = False
        print("moving back",actual_dist_h)
        
    elif actual_dist_h > fb_range[1]:
        fb = 30
        waypoint_flag = False
        print("moving forward",actual_dist_h)
    print("drone positioning values:",lr,fb,ud,yaw)
    return lr,fb,ud,yaw,waypoint_flag #left_right velocity, forward_back, up_down, yaw

def keyboard_control():
    try:
        speed = 100
        lr, fb, ud, yaw = 0,0,0,0

        if key == ord('w'):
            fb = speed
            print("Keyboard action forward")
            #tello.move_forward(30)
                    
        elif key == ord('s'):
            fb = -speed
            print("Keyboard action backward")
            #tello.move_back(30)           
        
        elif key == ord('a'):
            lr = -speed
            print("Keyboard action left")
            #tello.move_left(30)          
        
        elif key == ord('d'):
            lr = speed
            print("Keyboard action right")
            #tello.move_right(30)          
        
        elif key == ord('c'):
            yaw = speed
            print("Keyboard action clockwise")
            #tello.rotate_clockwise(10)          
        
        elif key == ord('z'):
            yaw = -speed
            print("Keyboard action anticlockwise")
            #tello.rotate_counter_clockwise(10)        
        
        elif key == ord('r'):
            ud = speed
            print("Keyboard action up")
            #tello.move_up(30)           
        
        elif key == ord('f'):
            ud = -speed
            print("Keyboard action down")
            #tello.move_down(30)
                
        #tello.send_rc_control(lr,fb,ud,yaw) #left_right velocity, forward_back, up_down, yaw

        return [lr,fb,ud,yaw]

    except:
        print("Command not accepted !")       


while True:    
    var1 = 0
    print("var1 set to", var1)

    while True:

        #drone telemetry
        drone_battery = tello.get_battery()
        drone_height = tello.get_height()

        #stream
        frame = tello.get_frame_read().frame
        
        cv2.putText(frame,f"Drone Battery: {drone_battery} %",(20,40),cv2.FONT_HERSHEY_PLAIN,1.3,(0,255,0),1,cv2.LINE_AA)
        cv2.putText(frame,f"Drone Height: {drone_height} cm",(20,60),cv2.FONT_HERSHEY_PLAIN,1.3,(0,255,0),1,cv2.LINE_AA)


        
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject  = aruco.detectMarkers(gray_frame, marker_dict, parameters = marker_param)

        for id, corners in zip(itertools.repeat(marker_IDs), marker_corners):
            cv2.polylines(frame, [corners.astype(np.int32)], True, (0,0,255),4,cv2.LINE_AA)

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

            cv2.circle(frame, (cx,cy),5,(0,0,255),-1)
            cv2.putText(frame,f"id:{id[0]}",top_left,cv2.FONT_HERSHEY_PLAIN,1.3,(0,0,0),2,cv2.LINE_AA)
            cv2.putText(frame,f"distance:{actual_dist_h}",(20,20),cv2.FONT_HERSHEY_PLAIN,1.3,(0,255,0),1,cv2.LINE_AA)         
                        



            #velocity command for positioning only if any id is in frame and keyboard control
            # if len(id)==0:
            #     lr, fb, ud, yaw, waypoint_flag = 0,0,0,0,False
            #     #lr, fb, ud, yaw = keyboard_control()
            # else:
            #     lr, fb, ud, yaw, waypoint_flag = drone_positioning()
            # lr, fb, ud, yaw = 0,0,0,0

        lr,fb,ud,yaw = keyboard_control()
        print("Rc command send", lr, fb, ud, yaw ) 
        tello.send_rc_control(lr,fb,ud,yaw)   #left-right, forward-backward, up-down, yaw

            
            # waypoint_flag = False
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
            
        cv2.imshow("Drone Feed", frame)


        key = cv2.waitKey(1) 

        if key == ord("q"): # ESC
            tello.streamoff()
            break

        try:
            drone_takeoff()
        except:
            print("drone takeoff failed !")

        try:
            drone_land()
        except:
            print("drone failed to land !")
                    
    print("Press q to exit")
    if key == ord("q"): # ESC
            tello.streamoff()
            break



    

#position set points
# tello.move_right(90) 
# time.sleep(1)

# tello.move_up(30)
# tello.move_left(90)
# tello.move_up(30)
# tello.move_right(90)
# tello.move_left(30)
        