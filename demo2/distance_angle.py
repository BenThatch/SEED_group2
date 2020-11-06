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

This script contains three functions. The function 'dist_and_angle' reads an
image and determines if there is an Aruco marker in the image. If there is
an Aruco marker present, the function returns a vector with the distance to
the marker and the angle in degrees between the camera axis and the beacon.
If no Aruco marker is found, return None.
If Aruco is in left half of image, the angle returned is positive.
If Aruco is in right half of image, the angle returned is negative.
The angle is the first element in the vector.
The distance (in mm) is the second element in the vector.
To show the image on screen for troubleshooting, uncomment the 3 lines of code
above the outer 'if' statement.

The function 'take_picture' sets the ISO (light sensitivity) and
auto-white-balance of the camera, and then takes a picture and saves the
image for future processing. If the image is too dark, increase ISO value.
The image is a jpg but can be changed to png if needed.

The function 'main' initiates an infinite loop, so that the camera
continuously takes a picture, detects the presence of an Aruco marker, and
returns a vector containing the angle between the camera axis and the beacon
as well as the distance in mm to the beacon.

To run this script, connect a camera to the Raspberry Pi and run the module.
"""

#Import the necessary packages:
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np
import cv2.aruco as aruco
from fractions import Fraction

#Function to return the quadrant that the Aruco marker is in:
def dist_and_angle(image):
    fov = 62.2 #Pi camera horizontal field of view in degrees, from camera specs
    f_exp = 1488 #focal length (pixels); determined through triangulation
    #markerLength = 165 #side length of my printed aruco marker is 165mm long
    markerLength = 60 #side length (mm) of aruco marker used in Demo 2
    img = cv.imread(image) #read image
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250) #dictionary of Aruco markers
    param = aruco.DetectorParameters_create() #default detection parameters
    grayImg = cv.cvtColor(img,cv.COLOR_BGR2GRAY) #grayspace makes detection easier
    #detect aruco markers:
    corners, ids, rejected = aruco.detectMarkers(grayImg,
                                     aruco_dict,parameters=param)

    imgCenterX = int((img.shape[1]))/2 #calculate horizontal center

##Uncomment the following 3 lines if you want an image on your screen to help
##with troubleshooting

##    cv.imshow('aruco',grayImg) #shows image on screen
##    cv.waitKey(5000) #waits 5 seconds
##    cv.destroyWindow('aruco') #destroys image window

    if ids == None: #No Aruco marker has been detected
        return None
    else: #An Aruco marker has been detected
        #determine the angle to the marker
        markerCenterX = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0])/4
        beaconAngle = ((imgCenterX-markerCenterX)/imgCenterX)*(fov/2)
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
        distance_x = (f_exp*markerLength)/(pixelWidthAvgX) #using triangulation
        distance_y = (f_exp*markerLength)/(pixelWidthAvgY) #using triangulation
        distance = (distance_x + distance_y)/2
    return [beaconAngle,distance]
    #return [beaconAngle,distance,distance_x,distance_y,pixelWidthAvgX,pixelWidthAvgY] #used in testing


#The following code was adapted from https://picamera.readthedocs.io/en/release-1.10/api_array.html#pirgbarray
#Function to take a picture and feed image into detect_marker function:
def take_picture():
    with PiCamera() as camera:
        with PiRGBArray(camera) as output:
            camera.iso = 400 #100 to 200 are good for daylight. 400 to 800 good for indoors
            camera.awb_gains = (Fraction(11,8),Fraction(39,32)) #set auto-white-balance
            camera.capture(output, 'rgb') #capture a picture from camera in RGB colorspace
            img = output.array #convert to NumPy array
            img = cv.cvtColor(img,cv.COLOR_RGB2BGR) #convert to BGR colorspace
            cv.imwrite('detect.jpg',img) #write image to file
            position = dist_and_angle('detect.jpg') #feed image into detect_marker function
    return position

#Main function to continuously take pictures and detect Aruco markers:
def main():
    while(1): #infinite loop
        position = take_picture() #capture image and determine quadrant of Aruco marker
        print(position)
        time.sleep(0.5) #adjust this to fit within time constraints of problem

main()



