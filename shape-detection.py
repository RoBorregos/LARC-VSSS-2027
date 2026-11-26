import cv2 as cv
import numpy as np


cap = cv.VideoCapture(2)

while True:
    ret, frame = cap.read()
    
    
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)    
    ret, th = cv.threshold(gray, 110, 255, cv.THRESH_BINARY)
    
    
    
    cv.imshow("Formas", th)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv.destroyAllWindows()