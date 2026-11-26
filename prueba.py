
import cv2 as cv
import numpy as np


def pelota():
     f
     return True


def calibration():
     f



#naranja harcodeado
naranja = np.uint8([[[0, 128, 255]]])
hsv_naranja = cv.cvtColor(naranja, cv.COLOR_BGR2HSV)

h = int(hsv_naranja[0, 0, 0])
h_bajo = max(h-10, 0)
h_alto = min(h+10, 179)


video = cv.VideoCapture(2)

while (True):
    
    ret, frame = video.read()
    if not ret:
            break

    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

#colores con rango

    
    hsv_naranja_lower_bound = (np.uint8([h_bajo, 100, 100]))
    hsv_naranja_upper_bound = (np.uint8([h_alto, 255, 255]))

    frame_naranja = cv.inRange(frame_hsv, hsv_naranja_lower_bound, hsv_naranja_upper_bound)
    
    cv.imshow('Frame', frame)
    cv.imshow('Naranja', frame_naranja)


    if cv.waitKey(1) & 0xFF == ord('q'):
        break

    

video.release()
cv.destroyAllWindows()