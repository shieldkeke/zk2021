# -*- coding: GBK -*-
import cv2
import os
import time
from get_img import MyVideoCapture
cap = MyVideoCapture("csi")
# cap = MyVideoCapture(0)
width = cap.width
height = cap.height
size = (int(width),int(height))   

fourcc = cv2.VideoWriter_fourcc(*'mp4v') # others: DIVX XVID MJPG X264 WMV1 WMV2 
path = os.getcwd() + "/video"
if not(os.path.exists(path)):
    os.makedirs(path)
print(path)

#create VideoWriter for writing vedio
out = cv2.VideoWriter( path + '/' + time.strftime(r"%Y-%m-%d_%H-%M-%S",time.localtime()) + '.mp4', fourcc, 24.0, size)
try: 
    while cap.isOpened():
        frame = cap.read()
        cv2.imshow('frame', frame)
        out.write(frame)
        if cv2.waitKey(1) == ord('q'):  #Q to exit
            break

    cap.release()
    out.release()
    cv2.destroyAllWindows()
except:
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print("cam closed")
