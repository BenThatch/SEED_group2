##Dan Parr and Ruiqi Sun
##EENG 350 SEED Lab
##Demo 2
##Aruco Marker Distance and Angle
##11/11/2020

"""
IMPORTANT: before using this script, measure the marker length (in mm) of the
edge of the aruco marker you will be using. Update the markerLength variable
in the dist_and_angle function. You will also need to update the value of
time.sleep in the main function to fit within given time constraints.

This script captures a video and determines if there is an Aruco marker in
the video frame. If there is an Aruco marker present, the function returns 3
values: 1) the distance to the marker in mm, 2) the whole number value of the
angle in degrees, 3) the decimal value of the angle to 2 decimal places.
If no Aruco marker is found, return None.
If Aruco is in left half of image, the angle returned is positive.
If Aruco is in right half of image, the angle returned is negative.
The whole number part of the angle is the first returned value.
The decimal part of the angle is the second returned value.
The distance (in mm) is the third returned value.
To show the image on screen for troubleshooting, uncomment the last few lines
of code.

To run this script, connect a camera to the Raspberry Pi and run the module.
"""

#Import the necessary packages:
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np
import cv2.aruco as aruco
import serial

#setup for serial communication
ser = serial.Serial('/dev/ttyACM0', 115200)
#Wait for connection to complete
time.sleep(3)

#Function to read serial
def ReadfromArduino():
    while (ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("serial output : ", line)
        except:
            print("Communication Error")

fov = 62.2 #Pi camera horizontal field of view in degrees, from camera specs
f_len = 589 #focal length (pixels); determined through triangulation
markerLength = 53 #side length (mm) of aruco marker used in Demo 2
#f_len = 615 #focal length for a different camera
#markerLength = 165 #side length of my printed aruco marker is 165mm long
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250) #dictionary of Aruco markers
param = aruco.DetectorParameters_create() #default detection parameters
cap = cv.VideoCapture(0) #capture a video
while(1):
    ret,frame = cap.read() #capture a frame
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) #convert to grayscale
    
    #detect aruco markers:
    corners, ids, rejected = aruco.detectMarkers(frame,
                                 aruco_dict,parameters=param)

    imgCenterX = int((frame.shape[1]))/2 #calculate horizontal center

    
    if ids != None: #An Aruco marker has been detected
    #determine the angle to the marker
        markerCenterX = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0])/4
        beaconAngle = ((imgCenterX-markerCenterX)/imgCenterX)*(fov/2)
#        wholeAngle = int(beaconAngle)
#        decimalAngle = int(round((beaconAngle - wholeAngle) * 100))
#        if decimalAngle < 0:
#            decimalAngle = decimalAngle * -1
    #pixel vertices of marker:
        Xvertex0 = corners[0][0][0][0]
        Xvertex1 = corners[0][0][1][0]
        Xvertex2 = corners[0][0][2][0]
        Xvertex3 = corners[0][0][3][0]
        Yvertex0 = corners[0][0][0][1]
        Yvertex1 = corners[0][0][1][1]
        Yvertex2 = corners[0][0][2][1]
        Yvertex3 = corners[0][0][3][1]
    #determine aruco width in pixels:
        widthListX = sorted([Xvertex0,Xvertex1,Xvertex2,Xvertex3])
        widthListY = sorted([Yvertex0,Yvertex1,Yvertex2,Yvertex3])
        pixelWidthX1 = abs(widthListX[0] - widthListX[3])
        pixelWidthX2 = abs(widthListX[1] - widthListX[2])
        pixelWidthY1 = abs(widthListY[0] - widthListY[3])
        pixelWidthY2 = abs(widthListY[1] - widthListY[2])
        pixelWidthAvgX = (pixelWidthX1+pixelWidthX2)/2
        pixelWidthAvgY = (pixelWidthY1+pixelWidthY2)/2
    #determine distance to object
        distance_x = (f_len*markerLength)/(pixelWidthAvgX) #using triangulation
        distance_y = (f_len*markerLength)/(pixelWidthAvgY) #using triangulation
        distance = 0.0393701*(distance_x + distance_y)/2
#        print(wholeAngle, decimalAngle, distance)
#        print(pixelWidthAvgX, pixelWidthAvgY)
        
        angleSend= "A"+ str(beaconAngle)
        distanceSend="D"+str(distance) + "\n"
        PositionAndAngleData= angleSend + distanceSend
        ser.write(PositionAndAngleData.encode())
        time.sleep(0.1)
        print(angleSend +" " + distanceSend)
        time.sleep(0.1)
        ReadfromArduino()


####Uncomment the following lines if you want an image on your screen to help
####with troubleshooting
##    cv.imshow('frame', gray)
##    if cv.waitKey(1) == ord('q'):
##        break
##cap.release()
##cv.destroyAllWindows()
    





