# coding=gbk
# cd yolov5-master
# sudo python detect_new.py
import argparse
import os
import sys
import numpy as np
import time
from ser import Ser
from pathlib import Path
import cv2
from get_img import MyVideoCapture
import torch
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.datasets import LoadImages
from utils.general import (LOGGER, check_img_size, check_requirements, non_max_suppression, print_args, scale_coords)
from utils.torch_utils import select_device, time_sync


def filt_name(name,cnt_detect):
    filt = False
    if filt:
        if cnt_detect<=24 and not(name=="dp" or name=="xh"):
            return False
        if cnt_detect>24 and cnt_detect<=48 and not(name=="lsfk" or name=="hskf" or name=="mf"):
            return False
        if cnt_detect>48 and not(name=="ad"):
            return False
    return True

def filt_y(res,cnt_detect):
    height = 480
    y = (res[1]+res[3])//2
    if y<height * 1/2:
        return False
    area = (res[3]-res[1])*(res[2]-res[0])
    if area < 7000 and cnt_detect<=48 :
        return False
    
    return True

def filt_conf(conf,cls):
    common = False
    if common:
        if conf<0.65:
            return False
    else:
        if cls=="mf":
            if conf<0.4:
                return False
        elif cls=="hsfk" or cls=="lsfk":
            if conf<0.6:
                return False
        else:
            if conf<0.9:
                return False
    return True

@torch.no_grad()
def run(weights=ROOT / 'best5-7.pt',  # model.pt path(s)
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=10,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        half=False,  # use FP16 half-precision inference
        ):
 

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device)
    stride, names, pt, jit, onnx, engine = model.stride, model.names, model.pt, model.jit, model.onnx, model.engine
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Half
    half &= (pt or jit or onnx or engine) and device.type != 'cpu'  # FP16 supported on limited backends with CUDA
    if pt or jit:
        model.model.half() if half else model.model.float()
    ## open camera ##
    cap = MyVideoCapture(0)
    cap.read()
    #img_path = "img.jpg"
    t = int(round(time.time()*1000))
    img_path = "my_data/"+f"{t}"+".jpg"  
    source = img_path

    #####serial#####
    cnt_com = 0
    com = Ser("/dev/ttyUSB0")
    #com = Ser('com9')
    #####competition rule####
    
    cnt_detect = 0
    while True:
        try:
            t = int(round(time.time()*1000))
            img_path = "my_data/"+f"{t}"+".jpg"  
            source = img_path  
           
            ch = com.get()
      
            if ch==b'@': 
                com.clearWait()
                print("begin to predict")

                cnt_detect += 1 #counter of detect

                img = cap.read()
                img = cv2.flip(img, -1)#trans 180
                cv2.imwrite(img_path, img)
				
                # Dataloader
                dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt)

                # Run inference
                model.warmup(imgsz=(1, 3, *imgsz), half=half)  # warm up
                dt, seen = [0.0, 0.0, 0.0], 0
                for path, im, im0s, _, s in dataset:
                    t1 = time_sync()
                    im = torch.from_numpy(im).to(device)
                    im = im.half() if half else im.float()  # uint8 to fp16/32
                    im /= 255  # 0 - 255 to 0.0 - 1.0
                    if len(im.shape) == 3:
                        im = im[None]  # expand for batch dim
                    t2 = time_sync()
                    dt[0] += t2 - t1

                    # Inference
                    visualize = False
                    pred = model(im, augment=augment, visualize=visualize)
                    t3 = time_sync()
                    dt[1] += t3 - t2

                    # NMS
                    pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
                    dt[2] += time_sync() - t3

                    # Process predictions
                    for _, det in enumerate(pred):  # per image # in fact , one pic for one cam
                        seen += 1
                        p, im0, _ = path, im0s.copy(), getattr(dataset, 'frame', 0)

                        p = Path(p)  # to Path
                        s += '%gx%g ' % im.shape[2:]  # print string

                        if len(det):
                            num = 0
                            # Rescale boxes from img_size to im0 size
                            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                            # Print results
                            for c in det[:, -1].unique():
                                n = (det[:, -1] == c).sum()  # detections per class
                                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                            # Write results
                            packet = f''
                            
                            for *xyxy, conf, cls in reversed(det):
                                res = torch.tensor(xyxy).numpy().astype(np.int32)
                                if filt_conf(conf,names[int(cls)]) and  filt_name(names[int(cls)],cnt_detect) and filt_y(res,cnt_detect):#confidence + filer by rule and size
                                    num = num + 1
                                    packet += f'{(res[0]+res[2])//2} '#add target x
                            packet = f'# {num} '+packet
                        else: 
                            packet = '# 0 '#nothinng

                        print(packet)#one pic one packet
                        ### send packet ###
                        com.put(packet)
                            
                            #    -------------->x
                            #    | x1,y1----
                            #    | |       |
                            #    | |----x2,y2 
                            #   \/
                            #    y
                        LOGGER.info(f'{s}Done. ({t3 - t2:.3f}s)')

        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            sys.exit(0)
        except Exception as e:
            print(e)
            cnt_com = 1 - cnt_com
            com = Ser("/dev/ttyUSB"+f"{cnt_com}")
            cnt_detect = 0
            #com = Ser('com9')

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default=ROOT / 'best5-7.pt', help='model path(s)')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=10, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(FILE.stem, opt)
    return opt


def main(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    run(**vars(opt))


if __name__ == "__main__":
    opt = parse_opt()
    main(opt)
