import sensor, image
import Message



def FindSquare():
    img = sensor.snapshot()
    
    for r in img.find_rects(threshold = 10000):
        img.draw_rectangle(r.rect(), color = (255, 0, 0))
        Message.UartSendData(Message.DotDataPack(1,1,r.x()-80,r.y()-60,Message.Ctr.T_ms))