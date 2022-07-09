import cv2, time, os
from get_img import MyVideoCapture
if __name__ =='__main__':
    cap = MyVideoCapture("csi")
    # cap = MyVideoCapture("0")
    try:
        if not os.path.exists(r"./data"):
           os.mkdir("data")
        while True:
            t = time.time()
            timestamp = int(round(t * 1000))
            img_path = f"data/{timestamp}.jpg"  

            time.sleep(1)   # every 1 second take a pic
            frame = cap.read()
            cv2.imwrite(img_path, frame)    
            print(img_path)
    except:
        cap.release()
        print("cam closed")
    
    

