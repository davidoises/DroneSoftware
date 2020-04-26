from tkinter import *
import serial
from time import sleep

def task():
    print(str(var.get()))
    message = "=" + str(var.get()) + "\n"
    #with serial.Serial('COM39', 115200) as ser:
    ser.write(str.encode(message))
    root.after(100, task)



ser = serial.Serial('COM38', 115200)

sleep(2)

root = Tk()
var = DoubleVar()
scale = Scale( root, variable = var, from_=2000, to=1000, resolution=1, length=250)
scale.pack(anchor = CENTER)

root.after(100, task)
root.mainloop()
