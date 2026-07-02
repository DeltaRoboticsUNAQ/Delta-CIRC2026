#!/usr/bin/env python3

import cv2

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

if ret:
    cv2.imwrite("captura.jpg", frame)
    print("Imagen guardada en captura.jpg")

cap.release()