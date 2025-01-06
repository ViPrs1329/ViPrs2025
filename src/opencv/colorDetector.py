# Python code for Multiple Color Detection 


import numpy as np 
import cv2 


# Capturing video through webcam 
webcam = cv2.VideoCapture(0) 

# Start a while loop 
while(1): 
    
    # Reading the video from the 
    # webcam in image frames 
    _, imageFrame = webcam.read() 
    webcamWidth = imageFrame.shape[1] / 2

    # Convert the imageFrame in 
    # BGR(RGB color space) to 
    # HSV(hue-saturation-value) 
    # color space 
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV) 

    # Set range for red color and 
    # define mask 
    red_lower = np.array([90,30,100], np.uint8) 
    red_upper = np.array([200,110,255], np.uint8) 
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 

    # Morphological Transform, Dilation 
    # for each color and bitwise_and operator 
    # between imageFrame and mask determines 
    # to detect only that particular color 
    kernel = np.ones((5, 5), "uint8") 
    
    # For red color 
    red_mask = cv2.dilate(red_mask, kernel) 
    res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                            mask = red_mask) 

    # Creating contour to track red color 
    contours, hierarchy = cv2.findContours(red_mask, 
                                        cv2.RETR_TREE, 
                                        cv2.CHAIN_APPROX_SIMPLE) 
    
    areasDict = []
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        approx = cv2.approxPolyDP( 
          contour, cv2.arcLength(contour, True), True) 
        if (area > 300): 
            areasDict.append({"area": area, "rect": cv2.boundingRect(contour)})
   
    def keyVal(item):
        # x, y, w, h = item["rect"]
        # center = x + w / 2
        # offset = abs(center - webcamWidth)
        # return offset
        return item["area"]

    if len(areasDict) != 0:
      areasDict = sorted(areasDict, key=keyVal)
      x, y, w, h = areasDict[-1]["rect"]
      imageFrame = cv2.rectangle(imageFrame, (x, y), 
                              (x + w, y + h), 
                              (0, 0, 255), 2) 
      cv2.putText(imageFrame, "Red Colour", (x, y), 
                  cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                  (0, 0, 255))   
            
    # Program Termination 
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame) 
    if cv2.waitKey(10) & 0xFF == ord('q'): 
        webcam.release() 
        cv2.destroyAllWindows() 
        break