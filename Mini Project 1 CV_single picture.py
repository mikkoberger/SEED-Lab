#--------------------------------------------------------------------------------------------------
#                                           DESCRIPTION OF CODE
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
#                                           INIT/IMPORTS
#--------------------------------------------------------------------------------------------------
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as  np
from smbus2 import SMBus
# This is the address we setup in the Arduino Program
address = 0x04

import time
import cv2
import cv2.aruco as aruco
import sys
import math
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

'''THIS BLOCK USED TO CREATE ARUCO MARKERS
#aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
#print(aruco_dict)
#img = aruco.drawMarker(aruco_dict, 4, 700) #aruco dictionary, ID of marker, size of marker
#cv2.imwrite("aruco id 4.jpg", img)
'''
#--------------------------------------------------------------------------------------------------
#                                           FUNCTIONS
#--------------------------------------------------------------------------------------------------


def read():
    encoder = bus.read_byte(address)
    print(encoder)
    rads=float(encoder)/10
    rads=str(rads)
    return rads

def write(data):
    bus.write_byte_data(address, 0, data)
    
def writeBlock(data):
    bus.write_i2c_block_data(address, 0, data)
    
def takePicture(fileName):

    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    rawCapture = PiRGBArray(camera)
 
    # allow the camera to warmup
    time.sleep(0.1)
     
    # grab an image from the camera
    print("Capturing Image...")
    try:
        camera.capture(rawCapture, format="bgr")
        image = rawCapture.array
        image=cv2.cvtColor(image, cv2.COLOR_BGR2RGB) #camera is capturing in BGR need to convert to RGB
        return image
    except:
        print("Failed to capture")
        
def showPicture(image,fileName): #display the image on screen and wait for a 'q' press
    cv2.imshow(fileName, image)
    
    
def cv_ex1_resize(image):
    #print("resizing image to 75% dimensions")
    image = cv2.resize(image,None,fx=0.75,fy=0.75) #decrease dimensions by 75%
    return image

def cv_ex7_calcDistance(fileName):
    image=takePicture(fileName)
    #image=cv2.imread(fileName)
    #image = image +15 #increases brightness of image
    image=cv_ex1_resize(image)#not necessary just loooks better for my monitor
    image=cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) #convert to Grayscale using a function for one line seems redundant
    arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejected = aruco.detectMarkers(image, arucoDict, parameters=parameters)
    aruco.drawDetectedMarkers(image, corners, ids)
    aruco.drawDetectedMarkers(image, rejected, borderColor=(100, 100, 100))

    if(ids == None):
        print("No markers found.")
        showPicture(image, fileName)
        return None
    else:
        for i in range(len(ids)):
            print("id found: %d" % ids[i])
 
    #-----------CALCULATE DISTANCE------------------------------------
    focalLength = 370 #mm.
    Y_markerHeight= 63.5 #mm
    pixmm=0.2645833333 #mm/pixel (from unitConverters.net) 
    for i in range(len(ids)): #getpixels that the corners are located at
        c = corners[i][0]
        center=[c[:, 0].mean()], [c[:, 1].mean()]
    #print(center)
    heightR=c[1, 1]-c[0, 1] #height in pixels of right side of aruco
    heightL=c[2, 1]-c[3, 1] #height in pixels of left side of aruco
    heightAvg=(heightL+heightR)/2 #get the average Height of both sides of marker (for accuracy's sake right?)
    
    #math based off: y=f*Y/Z from "camera vs real world" document
    y_imageHeight=heightAvg*pixmm #y
    distance=0.1*focalLength*Y_markerHeight/y_imageHeight #Z
    print("The aruco marker is %d cm away." % distance)
    
    #-----------CALCULATE ANGLE--------------------------------------
    picWidth=1440.0 #pixels
    picHeight=810.0 #pixels
    fovx=54 #degrees
    fovy=41
    
    xaxis=picWidth/2 #middle of image horizontally
    yaxis=picHeight/2  #middle of image vertically
    xdisp=center[0][0]-xaxis #displacement from center
    ydisp=center[1][0]-yaxis #y coords count from TOP down. so logic w y coords will be opposite
    
    cv2.line(image,(720,0),(720,810),(255,255,255),1)
    cv2.line(image,(0,410),(1440,410),(255,255,255),1)
    cv2.circle(image,(center[0][0],center[1][0]), 4, (255,255,255),1,-1)
    
    print('displacement from axis is:', xdisp, ydisp)
    quadrant=5 #not possible value for init
    if (xdisp>0 and ydisp<0):
        quadrant = 1 #top right (NE)
    elif (xdisp<0 and ydisp<0):
        quadrant = 2 #top left (NW)
    elif (xdisp<0 and ydisp>0):
        quadrant = 3 #bottom left (SW)
    elif (xdisp>0 and ydisp>0):
        quadrant = 4 #bottom right (SE)
    showPicture(image, fileName)    
    return quadrant
    
#--------------------------------------------------------------------------------------------------
#                                           MAIN
#--------------------------------------------------------------------------------------------------

quad=cv_ex7_calcDistance('test.jpg')
print('marker is in quadrant %d ' % quad)
desired = 0
if (quad== 1):
    desired = "1.57" #0
elif (quad== 2):
    desired = "3.14" #pi/2
elif (quad== 3):
    desired = "4.71"#bottom left (SW)
elif (quad== 4):
    desired = "6.28" #bottom right (SE)
desired=list(desired)
print('desired angle is %s ' % desired)

bus = SMBus(1)

print ("RPI: Hi Arduino, I sent you ", quad)

lcd.message = "target: "
lcd.cursor_position(9, 0)
for i in range(4):
    lcd.message = desired[i]
    lcd.cursor_position(i+10, 0)

   
write(quad)

t=0
while(cv2.waitKey(1) & 0xFF != ord('q')):
    current = read()

    lcd.cursor_position(0, 1)
    lcd.message = "current: "
    lcd.cursor_position(10, 1)
    for j in range(len(current)):
        lcd.message = current[j]
        lcd.cursor_position(j+11, 1)
       
    print ("Arduino: Hey RPI, I received: ", current)
    t+=1
print("exited")

bus.close()

cv2.waitKey(0)
cv2.destroyAllWindows()
