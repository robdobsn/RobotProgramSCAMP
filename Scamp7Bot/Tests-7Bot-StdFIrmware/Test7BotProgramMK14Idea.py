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

print("7Bot Test Programming MK14")
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
speeds = [100, 100, 100, 150, 150, 150, 150]
setSpeed(fluentEnables, speeds)

#keys = ["9", "8", "7", "6", "5", "4", "3", "2", "1", "0"]

#keys = ["MEM", "0", "0", "0", "0", "TRM", "D", "C", "TRM", "MEM", "TRM", "7", "4", "TRM"]

baseAddress = "0F12"
hexcodes = "C40D35C40031C401C8F4C410C8F1C400C8EEC40801C0E71EC8E49404C4619002C400C9808F01C0D89C0EC180E4FF9808C8CEC0CAE480C8C64003FC0194D6B8BF98C8C40790CE"

keyPositions = {
    "0": [11, 101, 104],
    "1": [7, 93, 104],
    "2": [14, 93, 104],
    "3": [18, 92, 104],
    "4": [7, 89, 100],
    "5": [13, 89, 100],
    "6": [17, 89, 100],
    "7": [7, 83, 97],
    "8": [11, 84, 97],
    "9": [16, 83, 97],
    "A": [23, 80, 97],
    "B": [28, 77, 97],
    "C": [33, 73, 97],
    "D": [27, 83, 100],
    "E": [32, 81, 100],
    "F": [36, 77, 100],
    "ABT": [26, 89, 104],
    "TRM": [34, 86, 104],
    "MEM": [29, 94, 105],
    "GO": [33, 90, 105],
}

servos3to7 = [83, 121, 122, 31]

# Set angles and wait for motion converge
print("Programming ...")

def sendKeySequence(keys):
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

def setAddress(addr):
    print("Setting address to", addr)
    keys = ["ABT", "MEM"]
    for ch in addr:
        keys.append(str(ch))
    sendKeySequence(keys)

def sendProgram(hexcodes):
    digitPairs = [hexcodes[i:i + 2] for i in range(0, len(hexcodes), 2)]
    for pair in digitPairs:
        keys = []
        keys.append("TRM")
        for ch in pair:
            keys.append(str(ch))
        keys.append("TRM")
        keys.append("MEM")
        print(pair)
        sendKeySequence(keys)

setAddress(baseAddress)
sendProgram(hexcodes)
setAddress(baseAddress)

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
