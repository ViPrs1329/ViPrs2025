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

    # Set range for teal color and 
    # define mask 
    teal_lower = np.array([0,0,0], np.uint8) 
    teal_upper = np.array([170,170,170], np.uint8) 
    teal_mask = cv2.inRange(hsvFrame, teal_lower, teal_upper) 
    # # Set range for green color and 
    # # define mask 
    # green_lower = np.array([25, 52, 72], np.uint8) 
    # green_upper = np.array([102, 255, 255], np.uint8) 
    # green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 

    # # Set range for blue color and 
    # # define mask 
    # blue_lower = np.array([94, 80, 2], np.uint8) 
    # blue_upper = np.array([120, 255, 255], np.uint8) 
    # blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 
    
    # Morphological Transform, Dilation 
    # for each color and bitwise_and operator 
    # between imageFrame and mask determines 
    # to detect only that particular color 
    kernel = np.ones((5, 5), "uint8") 
    
    # For teal color 
    teal_mask = cv2.dilate(teal_mask, kernel) 
    res_teal = cv2.bitwise_and(imageFrame, imageFrame, 
                            mask = teal_mask) 
    
    # # For green color 
    # green_mask = cv2.dilate(green_mask, kernel) 
    # res_green = cv2.bitwise_and(imageFrame, imageFrame, 
    #                             mask = green_mask) 
    
    # # For blue color 
    # blue_mask = cv2.dilate(blue_mask, kernel) 
    # res_blue = cv2.bitwise_and(imageFrame, imageFrame, 
    #                         mask = blue_mask) 

    # Creating contour to track teal color 
    contours, hierarchy = cv2.findContours(teal_mask, 
                                        cv2.RETR_TREE, 
                                        cv2.CHAIN_APPROX_SIMPLE) 
    
    areasDict = []
    for pic, contour in enumerate(contours): 
        area = cv2.contourArea(contour) 
        approx = cv2.approxPolyDP( 
          contour, cv2.arcLength(contour, True), True) 
        if (area > 300): 
            areasDict.append({"area": area, "rect": cv2.boundingRect(contour), "perim": cv2.arcLength(contour, True)})
            # x, y, w, h = cv2.boundingRect(contour) 
            # imageFrame = cv2.rectangle(imageFrame, (x, y), 
            #                         (x + w, y + h), 
            #                         (0, 0, 255), 2) 
            
            # cv2.putText(imageFrame, "teal Colour", (x, y), 
            #             cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
            #             (0, 0, 255))     

    def circularity(perim, area):
        return (4 * np.pi * area) / (perim * perim)
    
    def keyVal(item):
        # x, y, w, h = item["rect"]
        # center = x + w / 2
        # offset = abs(center - webcamWidth)
        # return offset
        return item["area"] * (circularity(item["perim"], item["area"]) ** 5)

    if len(areasDict) != 0:
      areasDict = sorted(areasDict, key=keyVal)
      x, y, w, h = areasDict[-1]["rect"]
      imageFrame = cv2.rectangle(imageFrame, (x, y), 
                              (x + w, y + h), 
                              (0, 0, 255), 2) 
      cv2.putText(imageFrame, "teal Colour", (x, y), 
                  cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                  (0, 0, 255))   
    # Creating contour to track green color 
    # contours, hierarchy = cv2.findContours(green_mask, 
    #                                     cv2.RETR_TREE, 
    #                                     cv2.CHAIN_APPROX_SIMPLE) 
    
    # for pic, contour in enumerate(contours): 
    #     area = cv2.contourArea(contour) 
    #     if(area > 300): 
    #         x, y, w, h = cv2.boundingRect(contour) 
    #         imageFrame = cv2.rectangle(imageFrame, (x, y), 
    #                                 (x + w, y + h), 
    #                                 (0, 255, 0), 2) 
            
    #         cv2.putText(imageFrame, "Green Colour", (x, y), 
    #                     cv2.FONT_HERSHEY_SIMPLEX, 
    #                     1.0, (0, 255, 0)) 

    # Creating contour to track blue color 
    # contours, hierarchy = cv2.findContours(blue_mask, 
    #                                     cv2.RETR_TREE, 
    #                                     cv2.CHAIN_APPROX_SIMPLE) 
    # for pic, contour in enumerate(contours): 
    #     area = cv2.contourArea(contour) 
    #     if(area > 300): 
    #         x, y, w, h = cv2.boundingRect(contour) 
    #         imageFrame = cv2.rectangle(imageFrame, (x, y), 
    #                                 (x + w, y + h), 
    #                                 (255, 0, 0), 2) 
            
    #         cv2.putText(imageFrame, "Blue Colour", (x, y), 
    #                     cv2.FONT_HERSHEY_SIMPLEX, 
    #                     1.0, (255, 0, 0)) 
            
    # Program Termination 
    # cv2.imshow("Multiple Color Detection in Real-TIme", res_teal) 
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame) 
    if cv2.waitKey(10) & 0xFF == ord('q'): 
        webcam.release() 
        cv2.destroyAllWindows() 
        break