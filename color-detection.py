import cv2 as cv
import numpy as np

def obtener_limites(color):
    if color == "rojo":
        return np.array([0, 100, 100]), np.array([10, 255, 255])
    elif color == "verde":
        return np.array([40, 100, 100]), np.array([80, 255, 255])
    elif color == "azul":
        return np.array([100, 100, 100]), np.array([140, 255, 255])

cap = cv.VideoCapture(2)

while True:
    ret, frame = cap.read()
    cv.imshow("Hola", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv.destroyAllWindows()