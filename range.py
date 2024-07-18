import serial
import threading
import psutil
import os
import RPi.GPIO as GPIO
from PCA9685 import PCA9685
from luma.core.interface.serial import i2c, spi
from luma.core.render import canvas
from luma.oled.device import ssd1309, ssd1325, ssd1331, sh1106
from time import sleep


distance = 0
strength = 0
angle = 10
running = True

def tonum(num):
    num = 2.5 + num/360 * 20
    return num

def rotate():
    global angle, running
    try:
        parent_pid = os.getppid()
        pwm = PCA9685()
        pwm.setPWMFreq(50)
        pwm.setRotationAngle(1, 10)
        angle = 11
        sleep(4)
        while running:
            if not psutil.pid_exists(parent_pid):
                print("rotate terminated")
                break
            for i in range(10,170,1): 
                pwm.setRotationAngle(1, i)
                angle = i
                sleep(0.03)

            for i in range(170,10,-1): 
                pwm.setRotationAngle(1, i)
                angle = i   
                sleep(0.03)
    except:
        sleep(2)

def read_tof():
    parent_pid = os.getppid()
    sleep(4)
    global distance, strength, running, angle
    ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
    ser.reset_input_buffer()

    while running:
        count = 0
        if not psutil.pid_exists(parent_pid):
            print("read terminated")
            break
        try:
            count = ser.in_waiting
        except:
            print("error")
            sleep(2)
        
        if count > 8:
            recv = ser.read(8)
            if recv[0] == 0x59 and recv[1] == 0x59:
                distance = recv[2] + recv[3] * 256
                strength = recv[4] + recv[5] * 256
                print(f'Distance: {distance} cm, Strength: {strength}, Angle: {angle}')
                ser.reset_input_buffer()
        sleep(0.5)
    ser.close()


def display_distance():
    parent_pid = os.getppid()
    serialC = i2c(port = 1, address = 0x3C)
    device = ssd1309(serialC)
    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, outline="blue", fill="black")
        draw.text((30,20),'starting...', fill="blue")
    sleep(4)
    global distance, strength, running
  
    print("start print")
    while running:
        if not psutil.pid_exists(parent_pid):
            print("display terminated")
            break
        with canvas(device) as draw:
            draw.text((30,20),f'Distance: {distance} cm', fill="blue")
        sleep(0.05)

if __name__ == "__main__":
    try:
        sleep(1)
        thread1 = threading.Thread(target = read_tof)
        thread2 = threading.Thread(target = display_distance)
        thread3 = threading.Thread(target = rotate)
        thread1.start()
        thread2.start()
        thread3.start()
        thread1.join()
        thread2.join()
        thread3.join()
    except KeyboardInterrupt:    
        running = False
