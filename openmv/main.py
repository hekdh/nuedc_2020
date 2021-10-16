#************************************ (C) COPYRIGHT 2019 ANO ***********************************#
import sensor, image, time, math, struct
import json
from pyb import LED,Timer
from struct import pack, unpack
import Message,LineFollowing,DotFollowing,ColorRecognition,QRcode,Photography,CircleFinder,SquareFinder
#初始化镜头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)#设置相机模块的像素模式
sensor.set_framesize(sensor.VGA)#设置相机分辨率160*120
sensor.set_windowing(0,240,640,100)
sensor.skip_frames(time=3000)#时钟
sensor.set_auto_whitebal(False)#若想追踪颜色则关闭白平衡

sensor.set_auto_gain(False)
clock = time.clock()#初始化时钟

#主循环
while(True):
    clock.tick()#时钟初始化
    #接收串口数据
    Message.UartReadBuffer()
    if Message.Ctr.WorkMode==1:#红色检测
        DotFollowing.DotCheck_r()
    elif (Message.Ctr.WorkMode==2):#绿色检测
        DotFollowing.DotCheck_g()
    elif Message.Ctr.WorkMode==3:#颜色识别
        ColorRecognition.ColorRecognition()
    elif Message.Ctr.WorkMode==4:#二维码识别
        QRcode.ScanQRcode()
    elif Message.Ctr.WorkMode==5:#拍照
        Photography.Photography('IMG.jpg',10)
        Message.Ctr.WorkMode = LastWorkMode
    LastWorkMode = Message.Ctr.WorkMode
    #用户数据发送
    #Message.UartSendData(Message.UserDataPack(127,127,32767,32767,65536,65536,65536,65536,65536,65536))
    #计算程序运行频率
    if Message.Ctr.IsDebug == 1:
        fps=int(clock.fps())
        Message.Ctr.T_ms = (int)(1000/fps)
        print('fps',fps,'T_ms',Message.Ctr.T_ms)

#************************************ (C) COPYRIGHT 2019 ANO ***********************************#
