import serial, time
from tkinter import *

ser = serial.Serial('/dev/ttyUSB0', 500000)

root = Tk()
root.title("PID Controller")

p_scale = Scale(root, from_=0, length=256, to=255, orient=HORIZONTAL)
p_scale.pack()

d_scale = Scale(root, from_=0, length=256, to=255, orient=HORIZONTAL)
d_scale.pack()

while True:
    ser.write([p_scale.get(), d_scale.get()])
    root.update()
    time.sleep(0.04)
