import os
import cv2 as cv

IMAGE = os.path.join(os.path.dirname(__file__), "Screenshot from 2026-08-17 17-20-00.png")

image = cv.imread(IMAGE)
print(type(image))