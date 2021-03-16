#!/usr/bin/env python3

import sys
import os
import numpy as np
import math
import cv2
from time import sleep
import serial
from mycalibrate import calibrate
from arduino_sucker import Arduino_Sucker

sys.path.append('/home/robotics/catkin_ws/devel/lib/python2.7/dist-packages')
import rospy
from tm_msgs.msg import *
from tm_msgs.srv import *

def mkdir(path):
    folder = os.path.exists(path)
    if not folder:
        os.makedirs(path)
        print('Create folder.')
    else:
        print(path+' folder exists.')

def send_script(script):
    rospy.wait_for_service('/tm_driver/send_script')
    try:
        script_service = rospy.ServiceProxy('/tm_driver/send_script', SendScript)
        move_cmd = SendScriptRequest()
        move_cmd.script = script
        resp1 = script_service(move_cmd)
    except rospy.ServiceException as e:
        print("Send script service call failed: %s"%e)
    
def set_io(state):
    rospy.wait_for_service('/tm_driver/set_io')
    try:
        io_service = rospy.ServiceProxy('/tm_driver/set_io', SetIO)
        io_cmd = SetIORequest()
        io_cmd.module = 1
        io_cmd.type = 1
        io_cmd.pin = 0
        io_cmd.state = state
        resp1 = io_service(io_cmd)
    except rospy.ServiceException as e:
        print("IO service call failed: %s"%e)

def set_waiting_mission(process_number):
    set_event(SetEventRequest.TAG, process_number, 0)

def wait_for_mission_complete(process_number):
    while True:
        rospy.sleep(0.2)
        res = ask_sta('01', str(process_number), 1)
        data = res.subdata.split(',')
        isComplete = data[1]
        if isComplete == "true":
            print("Waiting Complete.")
            break

def move_arm(X, Y, height, angle, speed):
    targetP1 = str(X) + " , " + str(Y) + " , " + str(height) + " , " + "180.00 , 0.00 , " + str(angle)
    script = "PTP(\"CPP\","+targetP1+","+ str(speed) +",200,0,false)"
    send_script(script)
    
def image_segmentation(bimg):
    Segmentation = []
    [height,width] = bimg.shape
    Region_count = 0
    Stack = []
    for h in range(height-1):
        for w in range(width-1):
            i = h
            j = w
            Stack = []
            Stack.append([i,j])
            Stack.append([0,0])
            if bimg[i,j] == 255:
                Region_count += 1
                bimg[i,j] = Region_count
                if(bimg[i,j+1] == 255):
                    bimg[i,j+1] = Region_count
                    Stack.append([i,j+1])
                if i > 1 and (bimg[i-1,j] == 255):
                    bimg[i-1,j] = Region_count
                    Stack.append([i-1,j]);   
                if j > 1 and (bimg[i,j-1] == 255):
                    bimg[i,j-1] = Region_count
                    Stack.append([i,j-1]);   
                if  bimg[i+1,j] == 255:
                    bimg[i+1,j] = Region_count
                    Stack.append([i+1,j])
                while not(Stack[-1] == [0,0]):
                    i = Stack[-1][0]
                    j = Stack[-1][1]
                    del(Stack[-1])
                    if not(i == 0 or j == 0 or i == height-1 or j == width-1):
                        if(bimg[i,j+1] == 255):
                            bimg[i,j+1] = Region_count
                            Stack.append([i,j+1])
                        if i > 1 and (bimg[i-1,j] == 255):
                            bimg[i-1,j] = Region_count
                            Stack.append([i-1,j]);              
                        if j > 1 and (bimg[i,j-1] == 255):
                            bimg[i,j-1] = Region_count
                            Stack.append([i,j-1]);             
                        if  bimg[i+1,j] == 255:
                            bimg[i+1,j] = Region_count
                            Stack.append([i+1,j])
    Seg = bimg.copy()
    needed_seg = []
    num_seg = bimg.max()+1
    for i in range(num_seg):
        area = np.sum(bimg == i)
        if area > 100  and area < 5000:
            needed_seg.append(i)
    for seg in needed_seg:
        segment = bimg == seg
        
        # centroid
        m00 = np.sum(segment)
        m10 = 0
        m01 = 0
        for x in range(height):
            m10 += x*np.sum(segment[x,:])
        for y in range(width):
            m01 += y*np.sum(segment[:,y])
        m10 = float(m10)
        m01 = float(m01)
        m00 = float(m00)
        xc = m10/m00
        yc = m01/m00
        
        # principal angle
        u20 = 0
        u02 = 0
        u11 = 0
        for x in range(height):
            for y in range(width):
                if(segment[x,y] == 1):
                    u20 += (x-xc)*(x-xc)
                    u02 += (y-yc)*(y-yc)
                    u11 += (x-xc)*(y-yc)
        phi = 0.5 * math.atan2(2*u11,u20-u02)
        phi *= 180/math.pi
        print("For segment %d, (xc,yc,phi) = (%.3f,  %.3f,  %.3f)" % (seg,xc,yc,phi))
        print("which has an area of %f." % m00)
        Segmentation.append([xc,yc,phi,m00,seg])
    return Segmentation


if __name__ == '__main__':
    try:
        ############## Connect to Arduino, Webcam, and Robot arm ####################

        # Arduino
        my_sucker = Arduino_Sucker()
        # my_sucker.connect()

        # Ros
        rospy.init_node('send_scripts', anonymous=True)
        set_event = rospy.ServiceProxy('tm_driver/set_event', SetEvent)
        ask_sta = rospy.ServiceProxy('tm_driver/ask_sta', AskSta)

        # Camera
        cap = cv2.VideoCapture(0)
        if not (cap.isOpened()):
            print('Could not open video device')
        else:
            print('Vedio device opened')

        ############## Show the image real-time ################
        while(True):
            ret,frame = cap.read()
            x = 206
            y = 253
            frame = cv2.circle(frame,(x,y),radius = 3,color=(0,0,255),thickness=-1)
            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('d'):
                break
        cv2.destroyAllWindows()
        ############################################### Camera Calibration ###############################################
        path = './calibration_params'
        mkdir(path)
        intrinsic_matrix = np.array([[839.51945494 ,  0     ,    296.47488036],
                                    [  0     ,    839.72471554 ,225.36108202],
                                    [  0     ,      0   ,       1        ]],dtype = np.float64)
        np.savetxt("./calibration_params/intrinsic_matrix.txt",intrinsic_matrix)
        dist_coeff = None
        object_points = np.array([
                                [450,250,0],
                                [300,300,0],
                                [200,400,0],
                                [350,450,0]
                                ],dtype = np.float64)
        np.savetxt("./calibration_params/object_points.txt",object_points)
        image_points = np.array([
                                [253.23,182.23],
                                [180.981,322.352],
                                [179.5,462.852],
                                [321.05,394.634]
                                ],dtype = np.float64)
        np.savetxt("./calibration_params/image_points.txt",image_points)
        print('Params saved.')
        calibration = calibrate(image_points, object_points, intrinsic_matrix, dist_coeff)

        # Use given set of points to verify the calibration.
        print('\n============Verifying the calibration=====================')
        pixel_point = np.array([250.888,106.918]) 
        world_point = np.array([500,200,0])

        # Transform the given world point into pixel point, and see if it match the given pixel point.
        guess_pixel_point = calibration.transform_world_to_pixel(world_point)
        print('Original pixel point : ',pixel_point)
        print('Calculated pixel point by a given world_point: ',guess_pixel_point)

        print('--------------------------\n')

        # Transform the given pixel point into world point, and see if it match the given pixel point.
        guess_world_point = calibration.transform_pixel_to_world(pixel_point)
        print('Original world point : ',world_point)
        print('Calculated world point by a given pixel_point: ',guess_world_point)
        print('================Verify end==========================\n')


        ############################################ Solving puzzle #############################################
        path2 = './segment_images'
        mkdir(path2)

        ret, frame = cap.read()
        
        # frame = cv2.imread('tmp.jpg')
        # save the original image
        cv2.imwrite('./segment_images/tmp.jpg',frame)

        #image processing, finding the centroid and principal angle
        # frame = cv2.imread('tmp.jpg')
        gframe = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        ret,bimg = cv2.threshold(gframe,200,255,cv2.THRESH_BINARY)
        bimg = bimg[0:500,:500]
        Segmentation = image_segmentation(bimg) # bimg will become segmentation image (pass by address)

        # save the every found segment (normalized)
        Out = bimg.copy()
        cv2.normalize(Out,Out,0,255,cv2.NORM_MINMAX)
        cv2.imwrite('./segment_images/every_segment.jpg',Out) # Found Segment, including those with large area.

        # save the filtered segment.(within particular area range)
        Out2 = np.zeros([bimg.shape[0],bimg.shape[1]])
        for i in range(len(Segmentation)):
            Out2 += (bimg == Segmentation[i][4])*(i+1)
        cv2.normalize(Out2,Out2,0,255,cv2.NORM_MINMAX)
        cv2.imwrite('./segment_images/listed_segment.jpg',Out2) # Listed Segment, only segments within particular area range are remained.
        
        # save image with centroids on it.
        centroid_img = frame.copy()
        for seg in Segmentation:
            xc = int(seg[0])
            yc = int(seg[1])
            centroid_img = cv2.circle(centroid_img,(yc,xc),radius = 3,color=(0,0,255),thickness=-1)
            text = str(seg[4])
            cv2.putText(centroid_img, text, (yc+10,xc+10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2,-1)# cv2.LINE_AA)
        cv2.imwrite('./segment_images/centroids.jpg',centroid_img)

        # choose your favorite segment.
        arr = input("\nWhich segments do you want to use? Enter 1 for the first segment: ")
        input_seg_list = [int(n)-1 for n in arr.split()]
        print("You've chosen %d segments in all." % len(input_seg_list))

        puzzle_result = []
        for index, seg in enumerate(input_seg_list):
            # In 'Segmentation', we have every listed segment with their features.
            xc = Segmentation[seg][0]
            yc = Segmentation[seg][1]
            phi = Segmentation[seg][2]
            i = (index % 4) 
            j = (index % 3)
            puzzle_result.append([xc, yc, phi, i, j])

        print("\nPuzzle result list: ", puzzle_result)
        print("\nPuzzle: [centroid_points (pixels), orientation_angle (degrees), row, col]")
        for puzzle in puzzle_result:
            print("Puzzle: ", puzzle)



        ########################################## Robot moving ################################################
        for index, puzzle in enumerate(puzzle_result):
            xc = puzzle[0]
            yc = puzzle[1]
            phi = puzzle[2]
            row = puzzle[3]
            col = puzzle[4]
            # From image
            suck_position = calibration.transform_pixel_to_world(np.array([xc, yc]))

            suck_x = suck_position[0]+1.4
            suck_y = suck_position[1]-0.8

            height = 220
            angle = 45
            speed = 100  # 100% of 5% (on the screen)

            OK_move = input("\n%d. Ready to go to suck a new puzzle (%d, %d) ? y/n: " %  ((index+1),row,col) )
            if OK_move == 'y':
                
                # Start sucking.
                # my_sucker.suck()

                # Move to (xc, yc) of the puzzle.
                height = 220
                move_arm(suck_x, suck_y, height, angle, speed)
                print("\n1. Move to that puzzle.")

                # Going down.
                height = 185 # sucker will kiss the puzzle at 188, and kiss the table at 185.
                move_arm(suck_x, suck_y, height, angle, speed)
                print("2. Going Down.")
                
                # Finished sucking, move up. 
                # height = 220
                # move_arm(suck_x, suck_y, height, angle, speed)
                # print("3. Finished sucking, move up. ")

                
            else:
                print("You rejected that puzzle.")

        
        cap.release()

    except rospy.ROSInterruptException:
        pass