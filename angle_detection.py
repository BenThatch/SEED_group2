##Dan Parr and Ruiqi Sun
##EENG 350 SEED Lab
##Demo 1
##Aruco Marker Angle
##10/26/2020

"""
This script contains three functions. The function 'marker_angle' reads an
image and determines if there is an Aruco marker in the image. If there is
an Aruco marker present, the function returns the angle in degrees between
the camera axis and the beacon.
If no Aruco marker is found, return None.
If Aruco is in left half of image, the angle returned is positive.
If Aruco is in right half of image, the angle returned is negative.
To show the image on screen for troubleshooting, uncomment the 3 lines of code
above the outer 'if' statement.

The function 'take_picture' sets the ISO (light sensitivity) and
auto-white-balance of the camera, and then takes a picture and saves the
image for future processing. If the image is too dark, increase ISO value.
The image is a jpg but can be changed to png if needed.

The function 'main' initiates an infinite loop, so that the camera
continuously takes a picture, detects the presence of an Aruco marker, and
returns the angle between the camera axis and the beacon.

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
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

lcd.clear()
# Set LCD color to red
lcd.color = [100, 0, 0]
time.sleep(1)

#Function to return the quadrant that the Aruco marker is in:
def marker_angle(image):
    fov = 62.2 #Pi camera horizontal field of view in degrees, from camera specs
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
        markerCenterX = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0])/4
        beaconAngle = ((imgCenterX-markerCenterX)/imgCenterX)*(fov/2)
    return beaconAngle


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
            angle = marker_angle('detect.jpg') #feed image into detect_marker function
    return angle

#Main function to continuously take pictures and detect Aruco markers:
def main():
    while(1): #infinite loop
        angle = take_picture() #capture image and determine quadrant of Aruco marker
        print(angle)
        if(angle == None):
            lcd.message="None";
            
        else:
             lcd.message= str(angle);
            
        time.sleep(2)        
        lcd.clear();       
        time.sleep(2) #adjust this to fit within time constraints of problem

main()



