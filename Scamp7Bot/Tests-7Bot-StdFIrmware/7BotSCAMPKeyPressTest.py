from __future__ import print_function
import serial
import numpy as np
import time
import threading

NUM_SERVOS = 7
isAllConverge = False
measuredForces = [0] * NUM_SERVOS
measuredRotationDegs = [0] * NUM_SERVOS

# Serial connection to 7Bot
serialIsClosing = False
botPort = serial.Serial(port="COM3", baudrate=115200, timeout=1)

print("7Bot Test Program")
time.sleep(4.0)
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
                #print("Forces:", measuredForces, ",Angles:", measuredRotationDegs, isAllConverge)
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

print("Going normal servo")
setForceStatus(1)

print("Set speed & pose")
setForceStatus(1)
time.sleep(1.0)

# Reboot 7Bot if previous status is not normal servo
# To make motion much more stable, highly recommend you use fluency all the time
fluentEnables = [True, True, True, True, True, True, True]
speeds_1 = [50, 50, 50, 100, 100, 100, 100]
setSpeed(fluentEnables, speeds_1)

#keys = ["9", "8", "7", "6", "5", "4", "3", "2", "1", "0"]

keys = ["MEM", "0", "0", "0", "0", "TRM", "D", "C", "TRM", "MEM", "TRM", "7", "4", "TRM"]

keyPositions = {
    "0": [12, 101, 104],
    "1": [7, 93, 104],
    "2": [13, 93, 104],
    "3": [21, 92, 104],
    "4": [7, 88, 102],
    "5": [13, 88, 102],
    "6": [20, 88, 102],
    "7": [7, 83, 100],
    "8": [11, 83, 100],
    "9": [16, 82, 100],
    "A": [23, 79, 100],
    "B": [28, 76, 100],
    "C": [34, 72, 100],
    "D": [24, 83, 102],
    "E": [28, 80, 102],
    "F": [35, 77, 102],
    "ABT": [26, 88, 104],
    "TRM": [33, 85, 104],
    "MEM": [27, 92, 105],
    "GO": [33, 90, 105],
}

servos3to7 = [83, 121, 122, 31]

# Set angles and wait for motion converge
print("Hover")

for key in keys:
    anglesDown = keyPositions[key] + servos3to7
    anglesUp = anglesDown[:]
    anglesUp[1] = anglesUp[1] + 6
    anglesUp[2] = anglesUp[2] - 7

    setServoAngles(anglesUp)
    while not isAllConverge:
        time.sleep(0.2)
    setServoAngles(anglesDown)
    while not isAllConverge:
        time.sleep(0.2)
    setServoAngles(anglesUp)
    while not isAllConverge:
        time.sleep(0.2)

    time.sleep(1)

# angles = [7, 102, 95, 83, 121, 122, 31]
# setServoAngles(angles)
# while not isAllConverge:
#     time.sleep(0.2)
#
# time.sleep(1)
#
# # Set angles and wait for motion converge
# print("Down")
# angles = [7, 96, 102, 83, 121, 122, 31]
# setServoAngles(angles)
# while not isAllConverge:
#     time.sleep(0.2)
#
# print("Up")
# angles = [7, 102, 95, 83, 121, 122, 31]
# setServoAngles(angles)
# while not isAllConverge:
#     time.sleep(0.2)

print("Clear")
angles = [7, 110, 80, 83, 121, 122, 31]
setServoAngles(angles)
while not isAllConverge:
    time.sleep(0.2)

time.sleep(0.5)

# Set angles and wait for motion converge
print("Home")
angles = [90, 115, 65, 90, 121, 122, 40]
setServoAngles(angles)
while not isAllConverge:
    time.sleep(0.2)


serialIsClosing = True
time.sleep(2.0)
botPort.close()
