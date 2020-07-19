import time

import board

import busio

import adafruit_adxl34x


import math 

i2c = busio.I2C(board.SCL, board.SDA)

accelerometer = adafruit_adxl34x.ADXL345(i2c)


fator_conv_rad_gra = 180/math.pi

while True:

    txt = "x{0:.2f} m/s\ty{1:.2f} m/s\tz{2:.2f} m/s"
    acceleration = [i/9.81 if i/9.81<1 and i/9.81>-1 else 0 for i in accelerometer.acceleration]
    print(acceleration)
    theta = math.atan(acceleration[1]/acceleration[0])*fator_conv_rad_gra
    phi = math.acos(acceleration[2]/math.sqrt(pow(acceleration[0],2) + pow(acceleration[1],2) + pow(acceleration[2],2)))*fator_conv_rad_gra

    print(txt.format(accelerometer.acceleration[0],accelerometer.acceleration[1],accelerometer.acceleration[2]),"phi: ",phi,"theta: ",theta)
    time.sleep(1)
