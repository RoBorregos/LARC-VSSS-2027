import cv2 as cv
import numpy as np
from PIL import Image

"""
def obtener_limites(color):
    if color == "rojo":
        return np.array([0, 100, 100]), np.array([10, 255, 255])
    elif color == "verde":
        return np.array([40, 100, 100]), np.array([80, 255, 255])
    elif color == "azul":
        return np.array([100, 100, 100]), np.array([140, 255, 255])
"""

naranja_bajo = np.array([10, 100, 20])
naranja_alto = np.array([25, 255, 255])


cap = cv.VideoCapture(2)

while True:
    ret, frame = cap.read()
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
    mask = cv.inRange(hsv, naranja_bajo, naranja_alto)

    mask_pillow = Image.fromarray(mask)
    bbox = mask_pillow.getbbox() 

    if bbox is not None:
        frame = cv.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 5)
    
    cv.imshow("Hola", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv.destroyAllWindows()