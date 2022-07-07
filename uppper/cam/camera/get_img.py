# coding:utf-8
'''
Author: CYK
Date: 2022-02-06 15:01:54
LastEditTime: 2022-02-06 20:47:31
LastEditors: Please set LastEditors
'''
import cv2
import numpy as np
import time
# 参数为0，调用内置摄像头，如果有其他的摄像头可以调整参数为1，2等
cap = cv2.VideoCapture(1)
cameraMatrix =np.matrix([[804.4703,-4.7160,404.5110],[0,799.1279,351.8036],[0,0,1]])
distCoeffs = np.matrix([[-0.5834],[0.7615],[0.0026],[0.0107],[0]])
R = np.identity(3)

while True:    
    # 从摄像头读取图片    
    success, img = cap.read()
    t = time.time()
    timestamp = int(round(t * 1000))    #毫秒级时间戳
    img_path = f"new_data/{timestamp}.jpg"  
    img_size = img.shape[:2]
    newCameraMatrix, _ = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, img_size, 1, img_size, 0)
    map1, map2 = cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, R, newCameraMatrix, img_size, cv2.CV_16SC2)
    rectified_img = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
    # 显示摄像头    
    cv2.imshow('----------please enter "s" to take a picture----------', rectified_img)    
    # 保持画面的持续,无限期等待输入    
    k = cv2.waitKey(1)    # k == 27 通过esc键退出摄像 ESC(ASCII码为27)    
    if k == 27:        
        cv2.destroyAllWindows()        
        break    
    elif k == ord("s"):        
        # 通过s键保存图片，并退出。        
        cv2.imwrite(img_path, rectified_img)           
# 关闭摄像头
cap.release()
cv2.destroyAllWindows()   