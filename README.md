# 项目说明

该项目为一个比赛赛项，要求在制定时间内遍历黑底白线的矩形模拟超市，抓取需要的商品，货架中会包含干扰物。本项目的机器人使用usb工业相机采集图像，在上位机jetson nano使用yolov5实现物品检测，可以实现10+帧率。传感器使用灰度传感器用于巡线，电机使用带AB相光电编码器的直流减速电机并调节pid。采用简易舵机机械臂作为抓取机构，步进电机控制器和丝杆导轨作为直线机构（用于增加机械臂的抓取范围）。使用电磁铁和永磁体连接机器人本体和购物车。

# 一些图片

设计图：



<img src="pics/pic0.jpg" style="zoom:8%;" />



校赛合影（队员为陈烨柯（右二）、余味（右一）、公冶在田（走了））：



<img src="pics/pic1.jpg" style="zoom:25%;" />

# 上位机文件夹upper使用说明：

1. yolov5文件夹内的文件放到 github上下载的yolov5文件夹中，其中pt模型换成自己的模型。当时是根据yolov5的某版本写的，torch版本1.10.0，torchvision 0.11.0a0。如果要其他版本，记得自行修改相应的地方。（好像找不到是哪个版本了，就顺便传上来github上yolov5的代码，在yolov5-from-github文件夹里，其中check_requirements函数被我注释掉了。）
2. 运行的时候用`sudo python detect_new.py`命令。
3. cam文件夹里是一些简单的相机功能，其中take_pic_period不需要开桌面，cam_video show_cam都需要开桌面
4. jetson nano csi相机的使用需要研究研究，可以参考[这篇文章](http://t.csdn.cn/CpoKG)，opencv 4.5.5亲测可以。

# 下位机文件夹lower使用说明：
1. 烧录到arduino mega

