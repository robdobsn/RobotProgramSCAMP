from __future__ import print_function
import serial
import numpy as np
import time
import threading
import msvcrt
from tkinter import *

master = Tk()

def grabAngles():
    global grabAnglesFlag
    grabAnglesFlag = True

def exitPgm():
    global globalAbortFlag
    globalAbortFlag = True

b1 = Button(master, text="GRAB", command=grabAngles)
b1.pack()
b2 = Button(master, text="EXIT", command=exitPgm)
b2.pack()

def waitAndFlush(timeInSecs):
    for tii in range(int(timeInSecs * 1000)):
        master.update_idletasks()
        master.update()
        sys.stdout.flush()
        time.sleep(0.001)
        if globalAbortFlag:
            break

grabAnglesFlag = False
globalAbortFlag = False
NUM_SERVOS = 7
isAllConverge = False
measuredForces = [0] * NUM_SERVOS
measuredRotationDegs = [0] * NUM_SERVOS

# Serial connection to 7Bot
serialIsClosing = False
botPort = serial.Serial(port="COM12", baudrate=115200, timeout=1)
#botPort = serial.Serial(port="/dev/ttyAMA0", baudrate=115200, timeout=1)

print("7Bot Test Program")
time.sleep(1)
print("Starting ...")

# Read data from bot
def botPortRead(ser):
    global isAllConverge, measuredRotationDegs, measuredForces, NUM_SERVOS
    global serialIsClosing
    rxBuf = [0] * (NUM_SERVOS * 2 + 1)
    beginFlag = False
    instruction = 0
    cnt = 0
    while True:
        if globalAbortFlag:
            break
        if serialIsClosing or not ser.isOpen():
            break
        val = ser.read(1)
        if len(val) == 0:
            continue
        # print("Rx", "%02X " % ord(val))
        if not beginFlag:
            beginFlag = (ord(val) == 0xFE)
            instruction = 0
            cnt = 0
        elif instruction == 0:
            instruction = ord(val) - 240
        elif instruction == 9:
            rxBuf[cnt] = ord(val)
            cnt += 1
            if cnt >= NUM_SERVOS * 2 + 1:
                beginFlag = False
                for i in range(NUM_SERVOS):
                    posCode = rxBuf[i*2] * 128 + rxBuf[i*2+1]
                    measuredForces[i] = posCode % 16384 / 1024
                    if posCode / 16384 > 0:
                        measuredForces[i] = -measuredForces[i]
                    # Convert 0-1000 code to 0-180 deg
                    measuredRotationDegs[i] = (posCode % 1024) * 9 / 50
                isAllConverge = (rxBuf[(NUM_SERVOS-1)*2+2] == 1)
                #print("Forces:", measuredForces)
                #print("Angles:", measuredRotationDegs)

        else:
            beginFlag = False

# Thread for reading from port
thread = threading.Thread(target=botPortRead, args=(botPort,))
thread.start()

def constrain(val, valMin, valMax):
    if val < valMin:
        return valMin
    if val > valMax:
        return valMax
    return val

# set motor force status: 0-forceless, 1-normal servo, 2-protection
def setForceStatus(status):
    data = bytearray([0xFE, 0xF5, status])
    botPort.write(data)

# set motion fluency & speeds (0~250 ---> 0~25)
def setSpeed(fluentEnables, speeds):
    # 1- Process Data
    sendData = bytearray([0xFE, 0xF7])
    servoIdx = 0
    for speed in speeds:
        sendData.append(constrain(speed, 0, 250)/10)
        if fluentEnables[servoIdx]:
            sendData[len(sendData)-1] += 64
        servoIdx += 1
    # 2- Send Data
    botPort.write(sendData)

def appendTwoByteVal(buf, val):
    buf.append((val / 128) & 0x7F)
    buf.append(val & 0x7F)

# set Servo angles
def setServoAngles(servoAngles):
    global isAllConverge
    isAllConverge = False
    # 1- Process Data
    sendData = bytearray([0xFE, 0xF9])
    for servoAngle in servoAngles:
        val = int(servoAngle*50/9)
        appendTwoByteVal(sendData, val)
    # 2- Send Data
    botPort.write(sendData)

def appendVecToSend(buf, vec):
    for el in vec:
        val = int(abs(el)) + (0 if el >= 0 else 1024)
        appendTwoByteVal(buf, val)

# IK6(6 angles)
# j6:mm(-500~500), vec:(-1.0~1.0)--->(-500~500), theta:Degrees
def setIK6(j6, vec56, vec67, theta6):
    global isAllConverge
    isAllConverge = False

    # 1- Process Data
    j6_c = np.array([constrain(j6[0], -500, 500), constrain(j6[1], -500, 500), constrain(j6[2], -500, 500)])
    vec56_c = np.copy(vec56)
    vec56_c /= np.linalg.norm(vec56_c)
    vec56_c *= 500
    vec67_c = np.copy(vec67)
    vec67_c /= np.linalg.norm(vec67_c)
    vec67_c *= 500

    sendData = bytearray([0xFE, 0xFA])
    appendVecToSend(sendData, j6_c)
    appendVecToSend(sendData, vec56_c)
    appendVecToSend(sendData, vec67_c)
    appendTwoByteVal(sendData, int((theta6*50/9)))

    # 2- Send Data
    # for dat in sendData:
    #     print("%02X " % dat, end = "")
    botPort.write(sendData)

print("Going forceless")
setForceStatus(0)
waitAndFlush(1)

keyIdx = 0
keySeq = ["0","1","2","3","4","5","6","7","8","9","A","B","C","D","E","F","TRM","MEM","ABT","GO"]
print("Move arms")
for i in range(1000):
    if globalAbortFlag:
        break
    print("Angles:", measuredRotationDegs)
    if grabAnglesFlag:
        with open("outangles.txt", "a") as myfile:
            outStr = '"' + keySeq[keyIdx] + '": ['
            outStr += ','.join(str(m) for m in measuredRotationDegs)
            outStr += '],\n'
            myfile.write(outStr)
            keyIdx += 1
            if keyIdx >= len(keySeq):
                keyIdx = 0
        grabAnglesFlag = False
    waitAndFlush(0.2)
print("Goodbye")

serialIsClosing = True
waitAndFlush(1)
botPort.close()
