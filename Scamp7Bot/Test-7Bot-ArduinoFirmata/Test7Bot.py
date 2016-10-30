import pyfirmata
from Tkinter import *

# don't forget to change the serial port to suit
board = pyfirmata.Arduino('COM3')

# start an iterator thread so
# serial buffer doesn't overflow
iter8 = pyfirmata.util.Iterator(board)
iter8.start()

# set up pin D9 as Servo Output
pin9 = board.get_pin('d:2:s')

def move_servo(a):
    pin9.write(a)

# set up GUI
root = Tk()

# draw a nice big slider for servo position
scale = Scale(root,
    command = move_servo,
    to = 180,
    orient = HORIZONTAL,
    length = 400,
    label = 'Base')
scale.pack(anchor = CENTER)

# run Tk event loop
root.mainloop()
