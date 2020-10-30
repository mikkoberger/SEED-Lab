
'''
Lucien Nuciola - Colorado School of Mines

This code is for the computer vision component of Demo 1 of SEED lab. the objective of this code is to continuously detect
an an aruco marker within the frame of the piCamera and report its angle along the x axis of the fram with respect to the y axis.
where left of the y axis is positively signed. the angle is then displayed to an LCD screen attached to the raspberry pi.

'''


#-----------------------IMPORTS/INIT--------------------------
import picamera 
import picamera.array 
import numpy as  np
# This is the address we setup in the Arduino Program
address = 0x04
from smbus2 import SMBus
import time
import cv2
import cv2.aruco as aruco
import sys
import math
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

#LCD INIT
lcd_columns = 16
lcd_rows = 2
# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)
# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()

#init to capture video and set variables
vid = cv2.VideoCapture(0)
picWidth=1440 #pixels    
picHeight=816 #pixels
newQuad=5 #not possible value for init
lastQuad=6
vid.set(3,picWidth)
vid.set(4,picHeight)


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
    
def demo1(): #report angle of aruco marker basically. this function will RETURN the angle and distance of the marker
    #initialization
    angle=0
    distance=0
    sideOfAxis = 3 #arbitrary initial value, variable used later to determine positive or negative angle
    cv2.waitKey(1) 
    ret, image = vid.read() #creates video capture object. 
    # alter the resulting frame to RGB then Grayscale
    image=cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image=cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    #aruco detection
    arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejected = aruco.detectMarkers(image, arucoDict, parameters=parameters)
    #draw borders on image identifying aruco markers
    aruco.drawDetectedMarkers(image, corners, ids)
    
    
    if(ids != None): #if an aruco marker is detected
        focalLength = 370 #mm.
        Y_markerHeight= 63.5 #mm
        pixmm=0.2645833333 #mm/pixel (from unitConverters.net)
        for i in range(len(ids)): #getpixels that the corners are located at
            c = corners[i][0]
            center=[c[:, 0].mean()], [c[:, 1].mean()] #calculate center pixel
        heightR=c[2, 1]-c[1, 1] #height in pixels of right side of aruco
        heightL=c[3, 1]-c[0, 1] #height in pixels of left side of aruco
        heightAvg=(heightL+heightR)/2 #get the average Height of both sides of marker (for accuracy's sake right?)
        y_imageHeight=heightAvg*pixmm#y
        distance=0.1*focalLength*Y_markerHeight/y_imageHeight #Z
    
        #-----------CALCULATE ANGLE/QUADRANT--------------------------------------
        fovx=54 #degrees. FOV in x axis of camera as listed on datasheet
        
        xaxis=picWidth/2 #middle of image horizontally
        xdisp=center[0][0]-xaxis #displacement from center
        maxWidth=distance*math.tan(math.radians(27.0)) #max width of image for a certain distance.
                                                       #aka given FOV of camera, width of image in pixels, pixels per mm for 96dpi
                                                       # and calculated distance of marker, we can determine how many cm in the x direction
                                                       # are visible in the image at that distance.
     
        if (xdisp>0): #right of axis
            xdist=(xdisp/xaxis) * maxWidth
            angle=-1*(180/np.pi)*(np.arctan(xdist/distance))
            #angle=-1*((center[0][0]/picWidth)-0.5)*fovx
        else:
            xdist=(center[0][0]/xaxis) * maxWidth
            angle=27-(180/np.pi)*(np.arctan(xdist/distance))
            #angle=(1-(center[0][0]/(picWidth/2)))*(fovx/2)           
        return angle, distance   
    else:
        return 'N/A  ','N/A  ' #if no marker is detected. 
       
'''
------------------------------MAIN-------------------------------------------------------
'''
#more LCD init, establishes communcation bus and prints Distance and Angle on
#Screen as we dont want to waste time refreshing that every time we loop later 
bus = SMBus(1)
lcd.cursor_position(0, 0)
lcd.message = "Distance:"
lcd.cursor_position(0, 1)
lcd.message = "Angle:"

#main loop for continuous operation
while True:
    ang,dist=demo1()  #call detection and calculation function. returns angle and distance 
    #if the angle returned is not a string, round to 2 decimal places.
    #if no marker is detected it will return a string and we dont want to round that hence the condition.
    if (type(ang)!=str): 
        ang= str(round(ang,2))
        dist = str(round(dist,2))
    print('angle: ', ang, '   distance: ', dist) #print angle and distance to the terminal
    
    #this block will write the angle and distance to the LCD screen
    lcd.cursor_position(10, 0)
    for i in range(len(dist)):
        lcd.cursor_position(i+10, 0)
        lcd.message = dist[i]
    lcd.cursor_position(10, 1)
    for j in range(len(ang)):
        lcd.cursor_position(j+10, 1)
        lcd.message = ang[j]
       
   


# Destroy all the windows 
cv2.destroyAllWindows() 


