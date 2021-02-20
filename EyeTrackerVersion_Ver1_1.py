# Filter Based Eye Tracking Script
# Original script author: Ryan H. (aka GunGryphon) 2020
# This script can be modified and used freely on terms of the accompanying MIT license

#V1.1 changes: Cleaned up commenting for release
#
#This script takes an input video feed of an eye and outputs a normalized eye position value.
#It's initial purpose is to drive an arduino based animatroinic eye mechanism. The physical eye mechanism 
#is driven using an arduino nano running a custom command parser.
#It requires a few calibration parameters to be calibrated, which can be set as defaults in the script.
#These parameters are also set during runtime using the UI sliders.
#Uncomment the respective video source to choose between webcam and a pre-recorded video file

 
##Import dependancies
import cv2
import numpy as np
import time
import math
import serial
import os
from datetime import datetime

####VARIABLES AND CONSTANTS-------------------------------------------------

##INPUT VIDEO FRAME PARAMETERS
FRAMEWIDTH = 640  #Default, changed once capture is open
FRAMEHEIGHT = 480

## ROI VARIABLES
#These dimensions are used to isolate the region of interest (ROI) from the rest of the frame.
#Set these just large enough to show the entire eye.
areaTop = 220
areaBottom = 480
areaLeft = 180
areaRight = 600

##FRAMERATE MONITOR VARIABLES
#These variables are used for calculating the script framerate for performance tuning
startTime = time.time()
endTime = time.time()
frameRate = 0.0

##ELIPSE PARAMETERS
#A blurred elipse is used to help remove bias in the software toward shadows under the eyelids
#Set elipse large enough to cover entire white area of eyes. Tuning may be required.
elpsOffset = (5,0)
elpsCenter = (int((areaRight - areaLeft)/2)+elpsOffset[0], int((areaBottom - areaTop)/2)+elpsOffset[1])
elpsAxisSize = (150,130)  #Elipse dimensions, tune these
elpsAngle = 0
elpsColor = (255,0,0)     #Color for overlay circle
elpsBlur = 25             #The blur changes the mask fallof rate towards the edges

##DETECTION AND AUTOTUNE VARIABLES
#The algorithm operates by isolating the darkest part of the image and comparing it's parameters to
#fit a few criteria.
threshVal = 45            #Base darkness threshold, pixels below this value are of interest
blurVal = 5               #Removes some noise, odd numbers only, lower = more fps
threshOffset = int(0)     #This value adjusts dynamically until only a certain % of pixels are unmasked
threshOffsetMax = 25      #Limit to brightness adjustment threshold
threshTarget = 0.018      #Ratio of pixels in ROI that we want to have left after darkness filter 
threshTargetLimits = 0.01 #Allowable difference in pixel ratio from target
threshAreaMax = 100       #Changes dynamically
error = 0.0               #Reports how large is the difference between our target and actual
minArea = 40              #Cut out contours smaller than this, they are probably noise


##POSITION PROCESSING
#To get a usable position value, we must map the image space position to a rotational eye value.
#This is usually set by manual run-time calibration.
absX = elpsCenter[0]      #Reference point for resting eye position
absY = elpsCenter[1]
relX = 0                  #Screen space position relative to resting eye position
relY = 0
minXY = [0,0]             #Image space limits of eye movement, used for mapping relative eye movement, set later
maxXY = [0,0]
centerXY = [0,0]
EBF = .75                 #Sets starting position min/max as a ratio of default elipse
normX = 0.5               #Normalized eye rotation values. Ranges 0 to 1 with 0.5 calibrated to resting position.
normY = 0.5               #These are probably the output values you are interested in.

##CONTOUR AND POSITION FILTER VALUES
#These are used to sort through the black/white image and choose the best candidate contour for the pupil
#Also includes positional other filters
blinkThreshCirc = 4.5     #Blinking causes the eye shadow to become much longer than the pupil,
blinkThreshAspect = 4.0   #These parameters set the blink detection limits
pupilAspect = 1           
pupilAspectThresh = 2.5
eyeClosedCnt = 0
eyeClosedThresh = 2       #When eyeclosed frames exceed this value, we are assuming a blink occurs.
eyeClosedLimit = 5        #EyeClosedCnt counts up and down to this value, used for limiting hysterisis
kernSize = 4              #Height of kernel filter, set 0 to disable
newWeight = 0.5           #A bias toward new data, higher places more weight on new frame data .5 = equal weight

##ANIMATRONIC INTERFACE VARIABLES
#This code was initially written to control an arduino based animatronic over USB serial
slaveSerialPort = '/dev/ttyUSB0' #Set to a valid serial port to enable animatronic interface, clear to disable
serialEn = False                 #Flag set if serial is online
slaveBaudRate = 115200
lidWidth = 0.4                   #This is used by the animatronic interface, eye open percentage is not currently tracked, but can be set manualy in UI


##OUTPUT IMAGE DISPLAY
#Controls whether image and overlays is drawn to screen.
realtimeDisplayEnable = True
trackbarDisplayEnable = True

##OUTPUT IMAGE RECORDING
#Annotated frames can be saved to file.
recording = False #For saving motion images
fileTime = 'NA'
frameNum = 1
outputFileLocation = "/home/pi/eyetrackoutput"


####FUNCTIONS-------------------------------------------------

### This function updates the filter parameters and recalculates
### masks when the controls are adjusted. Updating masks only when values
### are changed saves time in the core loop. 
def updateInputs(x):
    global areaTop
    global areaBottom
    global areaLeft
    global areaRight
    global threshVal
    global blurVal
    global elpsAxisSize
    global elpsCenter
    global elpsAngle
    global elipseMask
    global threshAreaMax
    global threshTarget
    global newWeight
    global kernSize
    print("Updating values")
    
    #update ROI bounds
    areaTop = cv2.getTrackbarPos('AreaTop','Controls')
    areaBottom = cv2.getTrackbarPos('AreaBottom','Controls')
    if areaBottom <= areaTop:
        areaBottom = areaTop + 1
    areaLeft = cv2.getTrackbarPos('AreaLeft','Controls')
    areaRight = cv2.getTrackbarPos('AreaRight','Controls')
    if areaRight <= areaLeft:
        areaRight = areaLeft + 1
    
    #update elipse bounds
    elpsAxisSize = (cv2.getTrackbarPos('ElipseWidth','Controls'), cv2.getTrackbarPos('ElipseHeight','Controls'))
    elpsAngle = cv2.getTrackbarPos('ElipseAngle','Controls')-45
    elpsOffset = (cv2.getTrackbarPos('ElipseOffsetX','Controls')-50, cv2.getTrackbarPos('ElipseOffsetY','Controls')-50)
    elpsCenter = (int((areaRight - areaLeft)/2)+(elpsOffset[0]), int((areaBottom - areaTop)/2)+(elpsOffset[1]))
    #Rebuild elipseMask
    elpsBlur = 1+(2*cv2.getTrackbarPos('ElipseBlur','Controls'))
    elipseMask = np.zeros((areaBottom - areaTop,areaRight - areaLeft,3), np.uint8)
    elipseMask.fill(255)
    cv2.ellipse(elipseMask, elpsCenter, elpsAxisSize, elpsAngle, 0,360, (0,0,0), -1)
    threshAreaMax = np.sum(elipseMask == 255)
    elipseMask = cv2.cvtColor(elipseMask, cv2.COLOR_BGR2GRAY)
    elipseMask = cv2.GaussianBlur(elipseMask, (elpsBlur,elpsBlur),0)
    
    #Update image detection thresholds
    threshVal = cv2.getTrackbarPos('Level','Controls')
    threshTarget = float(cv2.getTrackbarPos('AdaptiveMaskTarget','Controls')) /1000
    if threshTarget <= 0:
        threshTarget = .001
    blurVal = 1+(2*cv2.getTrackbarPos('Blur','Controls'))
    kernSize = cv2.getTrackbarPos('KernelFilter','Controls')
    newWeight = cv2.getTrackbarPos('NewDataBias','Controls')/100.0
    if newWeight < 0:
        newWeight = 0


###This function acts as the serial interface to the slave microcontroller.
###Any commands and operations must be matched in the microcontroller code.
def slaveCommand(opCode = 'x', arg0 = None, arg1 = None, arg2 = None, arg3 = None):
    global serialEn
    if serialEn:
        ser.write(opCode[0].encode())
        if arg0 != None:
            ser.write(str(int(arg0)).encode())
        if arg1 != None:
            ser.write(",".encode())
            ser.write(str(int(arg1)).encode())
        if arg2 != None:
            ser.write(",".encode())
            ser.write(str(int(arg2)).encode())
        if arg3 != None:
            ser.write(",".encode())
            ser.write(str(int(arg3)).encode())
        ser.write("\n".encode())
    return


###Wrapper function that formats and sends eye position commands to the slave device
def slaveMove(xval = 0.0, yval = 0.0, blink = 0,lidWidth = 0.3):
    global serialEn
    if serialEn:
        xCMD = int((1-np.clip(xval,0,1))*1023)
        yCMD = int((1-np.clip(yval,0,1))*1023)
        bCMD = int(np.clip(blink,0,1))
        lCMD = int((np.clip(lidWidth,0,1))*1023)
        slaveCommand('m',xCMD,yCMD,bCMD,lCMD)
    return

####SETUP-------------------------------------------------


##SETUP SERIAL
if os.path.exists(slaveSerialPort):         #If we find a valid serial port, enable serial, else skip
    ser = serial.Serial(port = slaveSerialPort,baudrate = slaveBaudRate, timeout = 10)
    if ser.isOpen:
        ser.close()
    ser.open()
    serialEn = True
    time.sleep(2)
    slaveCommand('a')
    time.sleep(1)
    ser.write("m90,90 \n".encode())
    time.sleep(1)
    ser.write("l\n".encode()) #Disable Arduino debug feedback
    
else:
    serialEn = False
    


##Setup video feed
#cap = cv2.VideoCapture("qsey+50evid1.avi") #Uncomment for prerecorded video and enter filename
cap = cv2.VideoCapture(0)                   #Uncomment for webcam, only use one capture source
if cap == None:
    print('Video failed to initialize')
    if serialEn:
        ser.close()
    sys.exit()   
FRAMEWIDTH = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
FRAMEHEIGHT = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)


##Create UI Sliders
img = np.zeros((40,512,3), np.uint8)
cv2.namedWindow('Controls')
cv2.createTrackbar('Level','Controls',threshVal,255,updateInputs)
cv2.createTrackbar('AdaptiveMaskTarget','Controls',int(threshTarget * 1000),200,updateInputs)
cv2.createTrackbar('Blur','Controls',int(blurVal/2),25,updateInputs)
cv2.createTrackbar('KernelFilter','Controls',kernSize,15,updateInputs)
cv2.createTrackbar('AreaTop','Controls',areaTop,int(FRAMEHEIGHT-1),updateInputs)
cv2.createTrackbar('AreaBottom','Controls',areaBottom,int(FRAMEHEIGHT),updateInputs)
cv2.createTrackbar('AreaLeft','Controls',areaLeft,int(FRAMEWIDTH-1),updateInputs)
cv2.createTrackbar('AreaRight','Controls',areaRight,int(FRAMEWIDTH),updateInputs)
cv2.createTrackbar('ElipseWidth','Controls',elpsAxisSize[0],int(FRAMEWIDTH/2),updateInputs)
cv2.createTrackbar('ElipseHeight','Controls',elpsAxisSize[1],int(FRAMEHEIGHT/2),updateInputs)
cv2.createTrackbar('ElipseOffsetX','Controls',elpsOffset[0]+50,100,updateInputs)
cv2.createTrackbar('ElipseOffsetY','Controls',elpsOffset[1]+50,100,updateInputs)
cv2.createTrackbar('ElipseAngle','Controls',(elpsAngle+45),90,updateInputs)
cv2.createTrackbar('ElipseBlur','Controls',elpsBlur,101,updateInputs)
cv2.createTrackbar('NewDataBias','Controls',int(newWeight*100),100,updateInputs)

##Create the Elipse mask to reduce eyelid interference
elipseMask = np.zeros((areaBottom - areaTop,areaRight - areaLeft,3), np.uint8)
elipseMask.fill(255)
cv2.ellipse(elipseMask, elpsCenter, elpsAxisSize, elpsAngle, 0,360, (0,0,0), -1) #Make the elipse on canvas
threshAreaMax = np.sum(elipseMask == 255)
elipseMask = cv2.cvtColor(elipseMask, cv2.COLOR_BGR2GRAY)
elipseMask = cv2.GaussianBlur(elipseMask, (elpsBlur,elpsBlur),0)                 #Blur the mask

##Set initial rough pupil bound references
minXY = [(int(elpsCenter[0] - (elpsAxisSize[0]*EBF))),(int((elpsCenter[1] - (elpsAxisSize[1]*EBF))))]
maxXY = [(int(elpsCenter[0] + (elpsAxisSize[0]*EBF))),(int((elpsCenter[1] + (elpsAxisSize[1]*EBF))))]
centerXY = [int((maxXY[0]+minXY[0])/2),int((maxXY[1]+minXY[1])/2)]

####CORE LOOP-------------------------------------------------
while True:
    startTime = time.time() #Starting frame time marker
    ret, frame = cap.read() #Get new image
    
    roi = frame[areaTop:areaBottom,areaLeft:areaRight]          #Set region of interest
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)            #Convert to grayscale
    gray_roi = cv2.GaussianBlur(gray_roi, (blurVal,blurVal),0)  #Blur to reduce noise
    gray_roi = cv2.add(gray_roi,elipseMask)                     #Overlay elipse mask to cut out external area
    
    ##Apply a threshold to mask the image, the loop cycles a few times until the 
    #percentage of white the pixels are within bounds. This accounts for common-mode lighting 
    BWMask = gray_roi        #Allocate mask 
    passCnt = 0
    while passCnt < 9:       #max attempts to get within specs, more attemps = possible frame stutter
        _, BWMask = cv2.threshold(gray_roi, (threshVal+ threshOffset), 255, cv2.THRESH_BINARY_INV)
        maskPercent = (np.sum(BWMask == 255)/threshAreaMax)  #What percentage of pixels are still within range?
        error = abs(maskPercent - threshTarget)/threshTarget #How far is our percentage from target
        if (maskPercent > threshTarget + threshTargetLimits and threshOffset > (-threshOffsetMax)): #Lower threshold
            threshOffset += -(1+int(0.3*error))
        elif (maskPercent < threshTarget - threshTargetLimits and threshOffset < threshOffsetMax):  #Raise threshold
            threshOffset += 1+int(0.3*error)
        else:
            break
        passCnt += 1
    print('Current White Pixel Ratio:', maskPercent)        
    print('Threshold Compensation: ',threshOffset)
    print('Error Value', error)
    
    #Kernel Filtering
    # if the eye is open, a vertically shaped kernel
    # will join disparate parts of the pupil together,
    # (usually due to horizontal screen glares)
    # if eye is closed, a wide kernel will join
    # eyelashes together to remove noise and add
    # histerysis. Small kernel = faster
    if kernSize > 0:
        if eyeClosedCnt > eyeClosedThresh:    #If the eyes are closed, change the kernel to a horizontal aspect
            kernel = np.ones((1,15),np.uint8)
        else:
            kernel = np.ones((kernSize,1),np.uint8)
        BWMask = cv2.dilate(BWMask, kernel, iterations = 1) #Dialate to stitch islands together
    
    ##Generate Contours from Masked Image
    _,contours, _ = cv2.findContours(BWMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse = True)    #Sort by size
    ##Find best contour
    cntIdx = -1                               #contour index, -1 means no valid contours were found 
    for i, cnt in enumerate(contours):    
        area = cv2.contourArea(cnt)
        if area < 500:                        #Skip small contours
            break
        (x, y, w, h) = cv2.boundingRect(cnt)  #Contour bounding box        
        if h == 0:                            #Check if contour data is valid
            break
        ##Contour shape check
        circum = cv2.arcLength(cnt, True)     #Circularity check
        circularity = (circum ** 2 / (4*3.14*area))
        pupilAspect = w/h
        if (pupilAspect > blinkThreshAspect): #The largest curve is too wide to be a pupil, skip loop
            break
        if (pupilAspect > pupilAspectThresh): #Still to wide to be a pupul, try next curve.
            continue
        cntIdx = i #If we get here, we must have the most promising pupil candidate
        
        ##UI contour overlay drawing
        ##Draw target
        cv2.drawContours(roi, contours[i], -1, (0,255,255), 2)       
        cv2.rectangle(roi,(x,y),(x + w, y + h),(0,0,255),2)
    
        ##Lowpass Filtering
        oldX = absX
        oldY = absY
        rawX = x + int(.5*w)
        rawY = y + int(.5*h)
        absX = int((rawX * newWeight) + (oldX * (1-newWeight)))
        absY = int((rawY * newWeight) + (oldY * (1-newWeight)))     
        
        ##Draw UI Items
        cv2.line(roi, (absX, 0), (absX,roi.shape[0]), (255,255,0),1)
        cv2.line(roi, (0, absY), (roi.shape[1],absY), (255,255,0),1)
        cv2.putText(roi,'Circ:   ' + str(circularity), (absX +10,absY), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,0,255),1)
        cv2.putText(roi,'Aspect: ' + str(pupilAspect), (absX +10,absY + 30), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,0,225),1)
        break
    
    ##Check if valid pupil was found, if not, count up eyes closed counter
    if cntIdx == -1:                      
        if eyeClosedCnt < eyeClosedLimit:  #No contours matched pupil, count up
            eyeClosedCnt+= 1
    elif eyeClosedCnt> 0:
        eyeClosedCnt+= -1

    ##Position Normalization
    #This maps the 2d image x and y values to a normalized value between 0 and 1 with 0.5 being neutral gaze.
    if absX >= centerXY[0]:                                                   #Map X to value based on center and bounds
        normX = np.clip(.5+((absX-centerXY[0])/(maxXY[0]-centerXY[0])/2),0,1)
    else:
        normX = np.clip(((absX-minXY[0])/(centerXY[0]-minXY[0])/2),0,1)
    if absY >= centerXY[1]:                                                   #Map Y to value based on center and bounds
        normY = np.clip(.5+((absY-centerXY[1])/(maxXY[1]-centerXY[1])/2),0,1)
    else:
        normY = np.clip(((absY-minXY[1])/(centerXY[1]-minXY[1])/2),0,1)
        
    ##Display Trackbar Interface
    if trackbarDisplayEnable:
        cv2.imshow('Controls',img)
    
    ##Draw ROI features on ROI image
    if realtimeDisplayEnable:
        #Bounding lines
        cv2.ellipse(roi, elpsCenter, elpsAxisSize, elpsAngle, 0,360, elpsColor, 2)
        if minXY[0] != 0:
            cv2.line(roi, (minXY[0], 0), (minXY[0],roi.shape[0]), (255,125,125),1)
        if maxXY[0] != 0:   
            cv2.line(roi, (maxXY[0], 0), (maxXY[0],roi.shape[0]), (125,255,125),1)
        if minXY[1] != 0:   
            cv2.line(roi, (0, minXY[1]), (roi.shape[1],minXY[1]), (255,125,125),1)
        if maxXY[1] != 0:
            cv2.line(roi, (0, maxXY[1]), (roi.shape[1],maxXY[1]), (125,255,125),1)      
        if centerXY[0] != 0:   
            cv2.line(roi, (centerXY[0], 0), (centerXY[0],roi.shape[0]), (125,125,255),1)
        if centerXY[1] != 0:   
            cv2.line(roi, (0, centerXY[1]), (roi.shape[1],centerXY[1]), (125,125,255),1)     
        #Position coordinate text
        cv2.putText(roi,("{:.4f}".format(normX)+' '+"{:.4f}".format(normY)), (20,50), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,0,255),2)
        if eyeClosedCnt>= eyeClosedThresh:
                cv2.putText(roi,'Eye Closed', (absX-100,absY-50), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),2)        #Draw blink text
        cv2.putText(roi,('FPS: '+"{:.4f}".format(frameRate)), (20,80), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,0,255),1) #Draw previous frame rate
        
        if recording:                                                                                     #Draw recording timestamps
            if fileTime == 'NA':                                                                                    
                fileTime = datetime.now().strftime("%Y%m%d%H%M%S")
            directory = "/home/pi/eyetrackoutput/ROIFrameRec/"+fileTime
            if not os.path.exists(directory):
                os.makedirs(directory)
            
            cv2.putText(roi,('Recording:' + datetime.now().strftime("%Y%m%d-%H:%M:%S.%f")), (20,areaBottom-areaTop -10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255),1)       
            filePre = directory +'/'+"ROIFrame_"
            _, preThresh = cv2.threshold(gray_roi, (threshVal), 255, cv2.THRESH_BINARY_INV)       
            cv2.imwrite(filePre+str(frameNum).zfill(4)+".jpg", roi)
            frameNum += 1


    ##Display images
    if realtimeDisplayEnable:
        cv2.imshow("Roi", roi)                   #This is the main image of interest
    ###cv2.imshow('ElipseMask',elipseMask)   ##The mask we add to the ROI to reduce eyelid interference
    ###cv2.imshow("Frame", frame)            ##Raw video frame before cropping
    ###cv2.imshow("Gray_Roi", gray_roi)      ##Greyscale ROI output
    ###cv2.imshow("BWMask", BWMask)          ##Final B/W image used for contour tracing


    ##Record frame calculation time
    endTime = time.time()
    cycleTime = endTime - startTime
    print("Frame Calc Time: ", "{:.1f}".format(cycleTime*1000),'ms')
    
    #Print eye location data
    print("Relative XY Position: ", "{:.3f}".format(normX),'X | ',"{:.3f}".format(normY),'Y')
    
    
    ##SERIAL OPERATIONS
    #This is where we command the output device to move to the new position
    if serialEn:
        while ser.in_waiting:                #Read out any feedback from slave device
            print(ser.readline())
        slaveMove(normX,normY,np.clip(eyeClosedCnt-(eyeClosedLimit+-1),0,1),lidWidth)

    
    ##UI key bindings
    #These keybinds are used to control and calibrate the program
    key = cv2.waitKey(1)
    if key == ord('q'):
        print ("Process Ended")
        break
    elif key == ord('w'):                      #Set pupil upper boundary
        minXY[1] = absY
    elif key == ord('s'):                      #Set pupil lower boundary
        maxXY[1] = absY
    elif key == ord('a'):                      #Set pupil leftmost boundary
        minXY[0] = absX
    elif key == ord('d'):                      #Set pupil rightmost boundary
        maxXY[0] = absX
    elif key == ord('c'):                      #Set resting gaze position (straight ahead)
        centerXY = [absX,absY]
    elif key == ord('i'):                      #Enable slave servos
        slaveCommand('a')
    elif key == ord('k'):                      #Disable slave servos
        slaveCommand('q')
    elif key == ord('y'):                      #Increase eye-open variable and slave write
        lidWidth = np.clip((lidWidth+0.1),0,1) 
        slaveCommand('w',int(lidWidth*1023))
    elif key == ord('h'):                      #Decrease eye-open variable and slave write
        lidWidth = np.clip((lidWidth-0.1),0,1) 
        slaveCommand('w',int(lidWidth*1023))
    elif key == ord('o'):                      #Toggle frame recording
        recording = not recording
        print("Recording" + str(recording))       
    elif key == ord('x'):                      #Toggle realtimeDisplayEnable
        realtimeDisplayEnable = not realtimeDisplayEnable
        print("Realtime Display Update Enable" + str(realtimeDisplayEnable))      
    elif key == ord('z'):                      #Toggle realtimeDisplayEnable
        trackbarDisplayEnable = not trackbarDisplayEnable
        print("Trackbar Drawing Update Enable" + str(trackbarDisplayEnable))     
    elif key == ord('p'):                      #Save screenshot of all images
        directory = outputFileLocation
        if not os.path.exists(directory):
            os.makedirs(directory)
        #os.chdir(directory)
        fileTime = datetime.now().strftime("%Y%m%d-%H%M%S")
        filePre = directory +"/"+"EyeTrack_"
        _, preThresh = cv2.threshold(gray_roi, (threshVal), 255, cv2.THRESH_BINARY_INV)
        cv2.imwrite(filePre+fileTime+"_1Frame.jpg", frame)
        cv2.imwrite(filePre+fileTime+"_2ROI.jpg", roi)
        cv2.imwrite(filePre+fileTime+"_3GrayROI.jpg", gray_roi)
        cv2.imwrite(filePre+fileTime+"_4ElipseMask.jpg", elipseMask)
        cv2.imwrite(filePre+fileTime+"_5PreThresh.jpg", preThresh)
        cv2.imwrite(filePre+fileTime+"_6BWMask.jpg", BWMask)
            
        
    ##Calculate actual framerate
    frameRate = 1/(time.time()-startTime)
    
#End of script, close all windows
cap.release()
cv2.destroyWindow('Controls')
cv2.destroyWindow('Roi')
if serialEn:
    ser.close()