#!/usr/bin/env python

# Python 2/3 compatibility
from __future__ import print_function

import cv2
import cv2 as cv
import urllib
import math
import numpy as np
from matplotlib import pyplot as plt
import argparse
import imutils
import glob
import serial
import serial.threaded
import time

import operator

import sys
import video

#Template 
stoptemp  = cv2.imread("stop_sign_template_small_01.png")
greentemp  = cv2.imread("green_traffic_light.png")
yellowtemp  = cv2.imread("yellow_traffic_light.png")
redtemp  = cv2.imread("red_traffic_light.png")
#fiftytemp  = cv2.imread("sign_speed-50")
#fivetemp  = cv2.imread("sign_speed-5")


stoptemp   = cv2.cvtColor(stoptemp, cv2.COLOR_BGR2GRAY)
greentemp  = cv2.cvtColor(greentemp, cv2.COLOR_BGR2GRAY)
yellowtemp  = cv2.cvtColor(yellowtemp, cv2.COLOR_BGR2GRAY)
redtemp  = cv2.cvtColor(redtemp, cv2.COLOR_BGR2GRAY)
#fiftytemp =  cv2.cvtColor(fiftytemp, cv2.COLOR_BGR2GRAY)
#fivetemp  = cv2.cvtColor(fivetemp, cv2.COLOR_BGR2GRAY)

templates = []

templates.append(stoptemp)
templates.append(greentemp)
templates.append(yellowtemp)
templates.append(redtemp)
#templates.append(fiftytemp)
#templates.append(fivetemp)



(tH0, tW0) = stoptemp.shape[:2]
(tH1, tW1) = greentemp.shape[:2]
(tH2, tW2) = yellowtemp.shape[:2]
(tH3, tW3) = redtemp.shape[:2]
#(tH4, tW4) = fiftytemp.shape[:2]
#(tH5, tW5) = fivetemp.shape[:2]





#Some values for setup
font = cv2.FONT_HERSHEY_SIMPLEX
wf = .2 #From 0 - 1, floating point. Low values favor old data, high values favor new data.
anglel = -1
angler = -1 #This value and its brother are -1 to make sure it doesn't smooth on first pass.
wpc = 1024 #Wheel Power Converter
scanresults = [0.0,0.0,0.0,0.0,0.0,0.0] #Scan result list
scanitems = ["Stop Sign","Green Light","Yellow Light","Red Light","55 Speed Limit SIgn","5 Speed Limit Sign","A whole lot of nothing"]
matchthresh = 0.070#threshold for matching
detsmoother = 0 #The old smoothed value for comparisions as to if we found a stop sign or not
#########################################################

#your tcp2serial server address and port (would be 127.0.0.1 if you are executing this py as the same RPi
host = "127.0.0.1"
port = "5555"
client_socket = socket.socket()


try:
    client_socket.connect((host, int(port)))
except socket.error as msg:
    sys.stderr.write('WARNING: {}\n'.format(msg))
    time.sleep(5)  # intentional delay on reconnection as client
    print("Socket Error... exiting")
    sys.exit(0)
    
    
sys.stderr.write('Connected\n')
client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

###################################################


if __name__ == '__main__':
    print(__doc__)

    try:
        fn = sys.argv[1]
    except:
        fn = 0

    def nothing(*arg):
        pass

    cv2.namedWindow('edge')
    cv2.namedWindow('edges2')
    cv2.createTrackbar('thrs1', 'edge', 100, 200, nothing)
    cv2.createTrackbar('thrs2', 'edge', 150, 200, nothing)

    cv2.createTrackbar('thrs1', 'edges2', 64, 200, nothing)
    cv2.createTrackbar('thrs2', 'edges2', 77, 200, nothing)
    
    cap = video.create_capture(fn)

while True:

#Get trackbar values
    cv2.namedWindow('edge')
    thrs1 = cv2.getTrackbarPos('thrs1', 'edge')
    thrs2 = cv2.getTrackbarPos('thrs2', 'edge')

    thrs3 = cv2.getTrackbarPos('thrs1', 'edges2')
    thrs4 = cv2.getTrackbarPos('thrs2', 'edges2')


#Read the Image
    flag, img = cap.read()
    height, width, channels = img.shape
    
    hw = width/2 # Half the width, or "hw"
    hh = height/2 # Half the height, or "hh"

#Convert to Grayscale, then crop to get our ROI (Bottom half of the screen)
    grayold = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = grayold[hh:height,0:width]
    grayresized =grayold[0:hh,0:width]
 
    edges = cv2.Canny(gray, thrs1, thrs2, apertureSize=3)
    edges2 = cv2.Canny(grayold,thrs3,thrs4)
    


#Cut our "edges" data into two halves, the left and right side of the screen.
    left = edges[0:height,0:hw]
    right = edges[0:height,hw:width]
#Define params for Hough Lines    
    minLineLength = 30
    maxLineGap = 10



#Perform Hough Lines for both the left and right side of the screen
    linesl = cv2.HoughLinesP(left,1,np.pi/180,100,minLineLength,maxLineGap)
    linesr = cv2.HoughLinesP(right,1,np.pi/180,100,minLineLength,maxLineGap)

    newanglel = 0 #This value and its brother represents our *new* values
    newangler = 0
    
    if linesl is not None:
        for x1,y1,x2,y2 in linesl[0]:
            cv2.line(img,(x1,y1+hh),(x2,y2+hh),(0,255,0),2)
            newanglel = math.atan2((y2-y1),(x2-x1))
            

    if linesr is not None:
        for x1,y1,x2,y2 in linesr[0]:
            cv2.line(img,(x1+hw,y1+hh),(x2+hw,y2+hh),(0,255,0),2)
            newangler = math.atan2((y2-y1),(x2-x1))
#Map Values (I want to see the angles in terms of 0 - 360 degrees)

    newangler = math.degrees(newangler)
    newanglel = math.degrees(newanglel)

#Scan for items and see which is best
        
    winner = 0
    newres = 0
    
    scanresults[0] =  cv2.matchTemplate(edges2, stoptemp, cv2.TM_CCOEFF_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(scanresults[0])

    top_left = max_loc
    bottom_right = (top_left[0] + tW0*2, top_left[1] + tH0*2)
    #This draws the rectangle  at the size of he template.
    cv.rectangle(img,top_left, bottom_right, 255, 2)

    wfs = .5 #weight for smoother
    detsmoother = (wf*max_val)+((1-wfs)*detsmoother)

#Smooth Values to reduce noise
    if anglel != -1:
        anglel = (wf*newanglel)+((1-wf)*anglel)
    else:
        anglel = newanglel

    if angler != -1:
        angler = (wf*newangler)+((1-wf)*angler)
    else:
        angler = newangler

#Deal with the stop sign we detected
    if detsmoother > matchthresh:#Stop a stop signs.
        anglel = 0
        angler = 0


#Map speed using angular distance from 90. 90 is up and down and the further you are from that
#the less power that wheel will get. For example, if there is a horizontal line on the left side and
#a vertical one on the right side, only the right will get power, and it will make a right turn.

#Performance on this depends on the camera as well...
    rwp = math.ceil(((abs(angler)/90))*wpc)
    lwp = math.ceil(((abs(anglel)/90))*wpc)


        
        
    
#Draw
    cv2.putText(img,'Left Angle ' + str(anglel),(10,50),font,0.75,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(img,'Right Angle ' + str(angler),(10,100),font,0.75,(255,255,255),2,cv2.LINE_AA)

    rwp = math.ceil(((abs(angler)/90))*wpc)
    lwp = math.ceil(((abs(anglel)/90))*wpc)

   
    
    cv2.putText(img,'Right Wheel: ' + str(rwp),(10,130),font,0.5,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(img,'Left Wheel: ' + str(lwp),(10,150),font,0.5,(255,255,255),2,cv2.LINE_AA)

    
    if detsmoother > matchthresh:
        cv2.putText(img,'Found a ' + scanitems[winner],(10,180),font,0.5,(255,225,100),2,cv2.LINE_AA)
    cv2.putText(img,str(detsmoother),(10,210),font,0.5,(255,225,100),2,cv2.LINE_AA)
   
#Show everyone our neat thing
    cv2.imshow('hough',img)
    cv2.imshow('edge', edges)
    cv2.imshow('edges2',edges2)
    cv2.imshow('Stopper',templates[0])

#Send speed information to  Serial
#The
    try:
        client_socket.send('<l'+str(int(lwp)).zfill(4)+'>')
        client_socket.send('<r'+str(int(rwp)).zfill(4)+'>')

    except socket.error as msg:
         print ("Sending error")

    
#Wait key business
    ch = cv2.waitKey(5)
    if ch == 27:
            break
