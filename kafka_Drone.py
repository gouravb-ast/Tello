

from djitellopy import Tello
import math, time
import cv2 
from cv2 import aruco
import numpy as np
import itertools
from kafka import KafkaProducer

#ArUCo
#marker_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
#param_marker = aruco.DetectorParameters_create()

producer = KafkaProducer(bootstrap_servers=['52.66.213.65:9092'],api_version=(0,10,1))


#Tello 
tello = Tello()
tello.connect()
tello.streamon()
frame_read = tello.get_frame_read()

def tello_takeoff():
    if key == ord('t'):
        tello.takeoff()

def tello_land():
    if key == ord('l'):
        tello.land()

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

        return lr, fb, ud, yaw

    except:
        print("Command not accepted !")
    
cap = cv2.VideoCapture(0)  
while True:
    # In reality you want to display frames in a seperate thread. Otherwise
    #  they will freeze while the drone moves.
    # 在实际开发里请在另一个线程中显示摄像头画面，否则画面会在无人机移动时静止
    img = frame_read.frame
    cv2.imshow("drone", img)
    
    ret, framex = cap.read() 
    print("a")
    ret, buffer = cv2.imencode('.jpg', framex)    
    producer.send("TestTopic", buffer.tobytes())
    # print("b")

    key = cv2.waitKey(1) & 0xff

    tello_takeoff()
    tello_land()

    # try:
    #     if key == ord('w'):
    #         #tello.move_forward(30)
    #         tello.send_rc_control(0,100,0,0) #left_right velocity, forward_back, up_down, yaw
        
    #     elif key == ord('s'):
    #         #tello.move_back(30)
    #         tello.send_rc_control(0,-100,0,0)            
        
    #     elif key == ord('a'):
    #         #tello.move_left(30)
    #         tello.send_rc_control(-100,0,0,0)            
        
    #     elif key == ord('d'):
    #         #tello.move_right(30)
    #         tello.send_rc_control(100,0,0,0)            
        
    #     elif key == ord('c'):
    #         #tello.rotate_clockwise(10)
    #         tello.send_rc_control(0,0,0,100)            
        
    #     elif key == ord('z'):
    #         #tello.rotate_counter_clockwise(10)
    #         tello.send_rc_control(0,0,0,-100)            
        
    #     elif key == ord('r'):
    #         #tello.move_up(30)
    #         tello.send_rc_control(0,0,100,0)            
        
    #     elif key == ord('f'):
    #         #tello.move_down(30)
    #         tello.send_rc_control(0,0,-100,0)

    #     elif key == ord('q'):
    #         break

    #     else:
    #         tello.send_rc_control(0,0,0,0)

    # except:
    #     print("Keyboard control failed !")

    lr,fb,ud,yaw = keyboard_control()
    tello.send_rc_control(lr,fb,ud,yaw)

    if key == ord("q"):
        break