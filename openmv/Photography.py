import sensor, image, pyb,time
import Message
RED_LED_PIN = 1
BLUE_LED_PIN = 3


class Video(object):
    VideoNumber = 0
    QRNumber = 0
#类的实例化
Video=Video()
def Photography(Photoname,SkipTime):
    img = sensor.snapshot()
    print("You're on camera!")
    pyb.LED(RED_LED_PIN).on()
    sensor.skip_frames(SkipTime)
    #红灯灭，蓝灯亮
    pyb.LED(RED_LED_PIN).off()
    pyb.LED(BLUE_LED_PIN).on()
    #保存截取到的图片到SD卡
    sensor.snapshot().save(Photoname)
    pyb.LED(BLUE_LED_PIN).off()
    print("Done! Reset the camera to see the saved image.")

    #航拍数据打包发送
   # Message.UartSendData(Message.VideoDataPack(Video.VideoNumber,Message.Ctr.T_ms))
