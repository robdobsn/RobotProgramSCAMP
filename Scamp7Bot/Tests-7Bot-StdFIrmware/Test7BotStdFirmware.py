from __future__ import print_function
import serial
import numpy as np
import time
import threading
import sys

# Read data from bot
def botPortRead(ser):
    global isAllConverge, measuredRotationDegs, measuredForces, NUM_SERVOS
    global serialIsClosing
    rxBuf = [0] * (NUM_SERVOS * 2 + 1)
    beginFlag = False
    instruction = 0
    cnt = 0
    while True:
        # Check for abort
#        checkForAbort()
        # Handle closing down
        if serialIsClosing or not ser.isOpen():
            break
        # Get a char if there is one
        val = ser.read(1)
        if len(val) == 0:
            continue
#        print("Rx", "%02X " % ord(val))
        if not beginFlag:
            beginFlag = (ord(val) == 0xFE)
            if not beginFlag:
                if (ord(val) < 0x20 or ord(val) > 0x7e) and ord(val) != 0x0d and ord(val) != 0x0a:
                    print("<%02X>" % ord(val))
                    sys.stdout.flush()
                else:
                    print(val.decode("utf-8"), end="")
                    sys.stdout.flush()
            instruction = 0
            cnt = 0
        elif instruction == 0:
            instruction = ord(val) - 240
        elif instruction == 6:
            forceStatus = ord(val)
            print("<== ForceStatus", forceStatus)
            beginFlag = False
            instruction = 0
            cnt = 0
        elif instruction == 9:
            rxBuf[cnt] = ord(val)
            cnt += 1
            if cnt >= NUM_SERVOS * 2 + 1:
                beginFlag = False
                instruction = 0
                cnt = 0
                for i in range(NUM_SERVOS):
                    posCode = rxBuf[i*2] * 128 + rxBuf[i*2+1]
                    measuredForces[i] = posCode % 16384 / 1024
                    if posCode / 16384 > 0:
                        measuredForces[i] = -measuredForces[i]
                    # Convert 0-1000 code to 0-180 deg
                    measuredRotationDegs[i] = (posCode % 1024) * 9 / 50
                isAllConverge = (rxBuf[(NUM_SERVOS-1)*2+2] == 1)
               # print("Forces:", measuredForces, ",Angles:", measuredRotationDegs, isAllConverge)
        else:
            beginFlag = False

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

# get servo angles
def getForceStatus():
    data = bytearray([0xFE, 0xF6, 0x00])
    botPort.write(data)

# set motion fluency & speeds (0~250 ---> 0~25)
def setSpeed(fluentEnables, speeds):
    # 1- Process Data
    sendData = bytearray([0xFE, 0xF7])
    servoIdx = 0
    for speed in speeds:
        sendData.append(constrain(speed, 0, 250)//10)
        if fluentEnables[servoIdx]:
            sendData[len(sendData)-1] += 64
        servoIdx += 1
    # 2- Send Data
    # Byte 1 (Beginning Flag) 0xFE
    # Byte 2 (Instruction Type) 0xF7 
    # Byte 3 (motor 0 data) 
    # Byte 4 (motor 1 data) 
    # Byte 5 (motor 2 data)
    # Byte 6 (motor 3 data)
    # Byte 7 (motor 4 data)
    # Byte 8 (motor 5 data)
    # Byte 9 (motor 6 data)
    # For each of bytes 3..9
    #     bit 6: 0-disable ?uency, 1-enable ?uency;
    #     bit 5~0: speed value(range:0~25, 10 means 100 degrees per second)
    botPort.write(sendData)

def appendTwoByteVal(buf, val):
    buf.append((val // 128) & 0x7F)
    buf.append(val & 0x7F)

# set Servo angles
def setServoAngles(servoAngles):
    global isAllConverge
    isAllConverge = False
    # 1- Process Data
    sendData = bytearray([0xFE, 0xF9])
    for servoAngle in servoAngles:
        val = servoAngle*50//9
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

def waitAndFlush(timeInSecs):
    for tii in range(int(timeInSecs * 1000)):
        sys.stdout.flush()
        time.sleep(0.001)

NUM_SERVOS = 7
isAllConverge = False
measuredForces = [0] * NUM_SERVOS
measuredRotationDegs = [0] * NUM_SERVOS

# Abort flag
globalAbortFlag = False

# Main program

print("7Bot Test Program")
print("Must be run in terminal - not IDLE as currently uses msvcrt (doesn't work under IDLE)")
print("Press ESC to abort")

# Serial connection to 7Bot
serialIsClosing = False
botPort = serial.Serial(port="COM12", baudrate=115200, timeout=1)
#botPort = serial.Serial(port="/dev/ttyAMA0", baudrate=115200, timeout=1)

# Thread for reading from port
thread = threading.Thread(target=botPortRead, args=(botPort,))
thread.start()

waitAndFlush(2)

print("____Setting normal servo")
setForceStatus(1)
waitAndFlush(1)

print("____Getting force status x 4")
getForceStatus()
waitAndFlush(1)
getForceStatus()
waitAndFlush(1)

# print("Going forceless")
# setForceStatus(0)
# time.sleep(5.0)
# print()
# print("____Going normal servo")
# setForceStatus(1)
# waitAndFlush(1)
# time.sleep(5.0)
# print("Going protection")
# setForceStatus(2)
# time.sleep(5.0)

print()
print("____Set speed")

# Reboot 7Bot if previous status is not normal servo
# To make motion much more stable, highly recommend you use fluency all the time
fluentEnables = [True, True, True, True, True, True, True]
speeds_1 = [30, 30, 30, 30, 50, 50, 50]
setSpeed(fluentEnables, speeds_1)
waitAndFlush(1)

# Set angles and wait for motion converge
print()
print("____Setting Angles 1")
angles_1 = [45, 115, 65, 90, 90, 90, 80]
setServoAngles(angles_1)
while not isAllConverge:
    waitAndFlush(0.1)
waitAndFlush(1)

# Set a angles and wait for motion converge
print()
print("____Setting Angles 2")
angles_2 = [135, 115, 65, 90, 90, 90, 80]
setServoAngles(angles_2)
while not isAllConverge:
    waitAndFlush(0.1)

# Speeds 2
speeds_2 = [50, 50, 50, 200, 200, 200, 200]
setSpeed(fluentEnables, speeds_2)

# Set angles and wait for motion converge
print()
print("____Setting Angles 3")
angles_3 = [45, 135, 65, 90, 90, 90, 80]
setServoAngles(angles_3)
while not isAllConverge:
    waitAndFlush(0.1)

# Set a angles and wait for motion converge
print()
print("____Setting Angles 4")
angles_4 = [135, 115, 65, 90, 90, 90, 80]
setServoAngles(angles_4)
while not isAllConverge:
    waitAndFlush(0.1)

# # IK Setting
# print("Setting IK6")
# j6 = np.array([0, 250, 150])
# vec56 = np.array([0, 0, -1])
# vec67 = np.array([1, 0, 0])
# setIK6(j6, vec56, vec67, 55)
# time.sleep(1.5)

print()
print("____Going forceless")
setForceStatus(0)
waitAndFlush(1)

serialIsClosing = True
waitAndFlush(2)
botPort.close()
