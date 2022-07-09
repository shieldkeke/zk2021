import cv2, queue, threading, time

# bufferless VideoCapture
class MyVideoCapture:

  def __init__(self, name):
    print("opening cam")
    if name=="csi":
      self.csi = True
      self.cap = cv2.VideoCapture(self.gstreamer_pipeline(flip_method = 0), cv2.CAP_GSTREAMER)
    else:
      self.csi = False
      self.cap = cv2.VideoCapture(name)
    self.width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)   
    self.height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)   

    if self.isOpened():
      print("cam opened")
    else:
      print("cam not opened")

    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()

  def release(self):
    self.cap.release()

  def isOpened(self):
    return self.cap.isOpened()
    
  def gstreamer_pipeline(
      self,
      sensor_id=0,
      capture_width=1920,
      capture_height=1080,
      display_width=960,
      display_height=540,
      framerate=30.0,
      flip_method=0,
  ):
      return (
          "nvarguscamerasrc sensor-id=%d !"
          "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
          "nvvidconv flip-method=%d ! "
          "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
          "videoconvert ! "
          "video/x-raw, format=(string)BGR ! appsink"
          % (
              sensor_id,
              capture_width,
              capture_height,
              framerate,
              flip_method,
              display_width,
              display_height,
          )
      )


if __name__ =='__main__':
    cap = MyVideoCapture(0)
    try:
      while True:
          time.sleep(.5)   # simulate time between events
          frame = cap.read()
          cv2.imshow("frame", frame)
          if chr(cv2.waitKey(1)&255) == 'q':
              break
      cap.release()
      cv2.destroyAllWindows()
      print("cam closed")
    except:
      cap.release()
      cv2.destroyAllWindows()
      print("cam closed")
    
