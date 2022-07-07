# -*- coding: UTF-8 -*-
import cv2
import os
import time

cap = cv2.VideoCapture(0)
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)   
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)  
size = (int(width),int(height))   
fourcc = cv2.VideoWriter_fourcc(*'mp4v') # ���������� DIVX��XVID��MJPG��X264��WMV1��WMV2��
path = os.getcwd() 
if not(os.path.exists(path)):
    os.makedirs(path)
print(path)

#����VideoWriter������д��Ƶ
out = cv2.VideoWriter( path + '/' + time.strftime(r"%Y-%m-%d_%H-%M-%S",time.localtime()) + '.mp4', fourcc, 24.0, size)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("No frame")
        break;
    cv2.imshow('frame', frame)
    out.write(frame)
    if cv2.waitKey(1) == ord('q'):  #��Q���˳�
        break

cap.release()
out.release()
cv2.destroyAllWindows()
