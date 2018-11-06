import cv2
import numpy as np

def nothing(x):
    pass

# Create a black image, a window
img = np.zeros((480,640,3), np.uint8)
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('Hmin','image',0,255,nothing)
cv2.createTrackbar('Hmax','image',255,255,nothing)
cv2.createTrackbar('Smin','image',0,255,nothing)
cv2.createTrackbar('Smax','image',255,255,nothing)
cv2.createTrackbar('Vmin','image',0,255,nothing)
cv2.createTrackbar('Vmax','image',255,255,nothing)

# create switch for ON/OFF functionality
switch = '0 : OFF \n1 : ON'
cv2.createTrackbar(switch, 'image',0,1,nothing)

inframe = cv2.imread("/home/pi/l0.jpg")
W = 400
height, width, depth = inframe.shape
imgScale = W/width
newX,newY = inframe.shape[1]*imgScale, inframe.shape[0]*imgScale
frame = cv2.resize(inframe,(int(newX),int(newY)))

while(1):

    # get current positions of four trackbars
    hmin = cv2.getTrackbarPos('Hmin','image')
    hmax = cv2.getTrackbarPos('Hmax','image')
    smin = cv2.getTrackbarPos('Smin','image')
    smax = cv2.getTrackbarPos('Smax','image')
    vmin = cv2.getTrackbarPos('Vmin','image')
    vmax = cv2.getTrackbarPos('Vmax','image')
    s = cv2.getTrackbarPos(switch,'image')
    
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # define range of blue color in HSV
    lower_blue = np.array([hmin, smin, vmin])
    upper_blue = np.array([hmax, smax, vmax])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # morph
    if s == 1:
        kernel = np.ones((5,5),np.uint8)
        dilation = cv2.dilate(mask,kernel,iterations = 2)
        kernel = np.ones((15,15),np.uint8)
        opening = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    cv2.imshow('image',frame)
    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
