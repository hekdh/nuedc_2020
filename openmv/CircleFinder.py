import sensor, image
import Message

# sensor.skip_frames(time = 2000)
def FindCircle():
    img = sensor.snapshot().lens_corr(1.8)

    for c in img.find_circles(threshold = 2000, x_margin = 10, y_margin = 10, r_margin = 10,
            r_min = 2, r_max = 100, r_step = 2):
        img.draw_circle(c.x(), c.y(), c.r(), color = (255, 0, 0))
        Message.UartSendData(Message.DotDataPack(0,1,c.x() - 80,c.y() - 60,Message.Ctr.T_ms))
