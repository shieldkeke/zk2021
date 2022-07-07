#coding=utf-8
import time
import serial 
import sys
import cv2
port = '/dev/ttyUSB0' #linux
#port = 'com9'#windows
class Ser:
    def __init__(self, _port='/dev/ttyUSB0', _bitrate = 115200, _timeout = 0.1):
        self.port = _port
        self.bitrate = _bitrate
        self.timeout = _timeout
        self.connecting = False
        self.connect()

    def connect(self):
        while not self.connecting:
            try:
                time.sleep(1)
                print("try")
                self.com = serial.Serial(self.port, self.bitrate, timeout=self.timeout) 
                self.connecting = True
                print("ok")
                break
            except KeyboardInterrupt:
                print("KeyboardInterrupt")
                sys.exit(0)
            except:
                print("fail")

    def info(self):
        print(self.com)
    
    def setPort(self, _port):
        self.port = _port
        self.connect()

    def setBitrate(self, _bitrate):
        self.bitrate = _bitrate
        self.connect()

    def setTimeout(self, _timeout):
        self.timeout = _timeout
        self.connect()
    
    def available(self):
        return self.connecting
        
    def clearWait(self):
        while self.com.inWaiting():
            self.com.read()
    
    def isOpen(self):
        return self.com.is_open

    def get(self):
        ch = ' '
        if self.com.inWaiting():
            ch = self.com.read()
            print(ch)
        return ch
    
    def put(self, str):
        print("writing:"+str)
        self.com.write(str.encode("utf-8"))
        print("done")
    
def detect():
    print("begin to predict")
    t = int(round(time.time()*1000))
    img_path = "my_data/"+f"{t}"+".jpg"  
    _, img = cap.read()
    #cv2.imshow(img)
    cv2.imwrite(img_path, img)
    print("end")
    time.sleep(5)
    
if __name__ =='__main__':
    com = Ser(port)
    cap = cv2.VideoCapture(0)
    while True:
        if com.get()==b'@':
            detect()
            #com.put("# 10 2 3 5 7 31 25 68 24 2 3 5 7 31 25 68 24 31 25 68 24 2 3 5 7 31 25 68 24 2 3 5 7 31 25 68 24 31 25 68 9 ")
            #com.put("# 40 2 3 5 7 31 25 68 24 2 3 5 7 31 25 68 24 31 25 68 24 2 3 5 7 31 25 68 24 2 3 5 7 31 25 68 24 31 25 68 9 ")
            com.put("# 0 ")
        #break
