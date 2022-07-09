# coding:utf-8

import cv2
import time
import os 
from get_img import MyVideoCapture
cap = MyVideoCapture('csi')

try:
    while True:    

        img = cap.read()
        t = time.time()
        timestamp = int(round(t * 1000))
        img_path = f"data/{timestamp}.jpg"  
        cv2.imshow('----------please enter "s" to take a picture----------', img)    
        k = cv2.waitKey(1) 
        if k == 27:        
            cv2.destroyAllWindows()        
            break    
        elif k == ord("s"):        
            if not os.path.exists(r"./data"):
                os.mkdir("data")
            cv2.imwrite(img_path, img)           

    cap.release()
    cv2.destroyAllWindows()   
except:
    cap.release()
    cv2.destroyAllWindows()
    print("cam closed")
