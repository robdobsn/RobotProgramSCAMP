import pyfirmata
from Tkinter import *

# don't forget to change the serial port to suit
try:
    robotCtrl = pyfirmata.Arduino('COM3')
except:
    print("Cannot open robot port")
    exit(0)

# start an iterator thread so
# serial buffer doesn't overflow
# iter8 = pyfirmata.util.Iterator(robotCtrl)
# iter8.start()

# servoList = [
#     {"pin": 2, "min": 0, "max": 180, "name": "base", "initial": 90},
#     {"pin": 3, "min": 0, "max": 180, "name": "lift", "initial": 150},
#     {"pin": 4, "min": 0, "max": 180, "name": "base"},
#     {"pin": 5, "min": 0, "max": 180, "name": "base"},
#     {"pin": 6, "min": 0, "max": 180, "name": "base"},
#     {"pin": 7, "min": 0, "max": 180, "name": "base"},
# ]
# # set up servo outputs
# for servoItem in servoList:
#     servoItem["output"] = robotCtrl.get_pin('d:' + str(servoItem["pin"]) + ':s')
#
# def moveServo(servoItem, a):
#     servoItem["output"].write(a)

# Move to initial positions
# for servoItem in servoList:
#     if "initial" in servoItem:
#         moveServo(servoItem, servoItem["initial"])

# set up GUI
# tkRoot = Tk()

# Exit fn
# def exitCallback():
#     tkRoot.destroy()
#     tkRoot.quit()

# Exit button
# exitButton = Button(tkRoot, text="Exit", command=exitCallback)
# exitButton.pack()

# draw servo sliders
# scale0 = Scale(root,
#                 command = moveServo0,
#                 to = 180,
#                 orient = HORIZONTAL,
#                 length = 400,
#                 label = servoList[0]["name"])
# scale0.pack(anchor = CENTER)
# scale0.set(servoList[0]["initial"])
# scale1 = Scale(root,
#                command=moveServo1,
#                to=180,
#                orient=HORIZONTAL,
#                length=400,
#                label=servoList[1]["name"])
# scale1.pack(anchor=CENTER)
# scale1.set(servoList[1]["initial"])

# run Tk event loop
# tkRoot.mainloop()

print("here")
robotCtrl.exit()

