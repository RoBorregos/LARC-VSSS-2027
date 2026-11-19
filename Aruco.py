import cv2 as cv

parametros = cv.aruco.DetectorParameters_create()
diccionario = cv.aruco.Dictionary_get(cv.aruco.DICT_5X5_1000)

cap = cv.VideoCapture(0)
while True:
    si, fotograma = cap.read()
    if not si:
        break
    gris = cv.cvtColor(fotograma, cv.COLOR_BGR2GRAY)
    corners, ids, rechazados = cv.aruco.detectMarkers(gris, diccionario, parameters=parametros)

    if ids is not None:
        cv.aruco.drawDetectedMarkers(fotograma, corners, ids)
            
    cv.imshow("ArUco", fotograma)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv.destroyAllWindows()  