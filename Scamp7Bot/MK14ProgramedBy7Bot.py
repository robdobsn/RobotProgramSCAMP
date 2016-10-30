# Program to program a Science of Cambridge MK14 with a 7Bot
# An MK14 is a very old micro-computer from what became Sinclair Research
# A 7Bot is a 7 degrees of freedom robot arm which orignated here:
# More information on this project is here: http://robdobson.com/2016/10/mk14-meets-7bot/

from __future__ import print_function
import serial
import numpy as np
import time
import threading
import sys
import tkinter

# Name of the program to "send" to the MK14 (entered physically by robot arm)
programToRun = "Duck Shoot"

# Setup of the 7Bot position and speed in relation to the MK14
normalSpeeds = [80, 80, 80, 150, 100, 100, 100]
homePositionAngles = [10, 150, 75, 83, 121, 89.64, 56]
readyPositionAngles = [7, 115, 65, 83, 121, 89.64, 56]
keyboardClearAngles = [7, 107, 90, 90, 118, 89.64, 56]

# Distance to move between hovering over a key and pressing it
# the three values are azimuth (0 means no change), shoulder position (+ve value) and
# elbow position (-ve value)
keyPunchDownAngleDeltas = [0, 5, -5]

# Enter a sequence here to override sending the MK14 a program and instead just press keys
testKeySequence = []
#testKeySequence = ["0","1","2","3","4","5","6","7","8","9","A","B","C","D","E","F","TRM","MEM","ABT","GO"]

# Positions of keys on the MK14 keypad
keyPositions = {
    "0": [12, 103-2, 102+2, 90, 115.56, 89.64, 56],
    "1": [7.92, 94+2, 103.52-2, 84.06, 116.1, 89.46, 56],
    "2": [12.6, 95, 101.52, 83.52, 116.64, 89.46, 56],
    "3": [18.2, 95, 102, 90, 114.3, 89.46, 56],
    "4": [7.2, 89, 101-0.5, 81.0, 117.9, 89.46, 56],
    "5": [11.7, 87+1.5, 102-1.5, 79.92, 118.8, 89.46, 56],
    "6": [15.5, 86+1, 102-1, 79.92, 120.42, 89.46, 56],
    "7": [9, 84, 101, 90, 114, 90, 56],
    "8": [13, 81, 100, 90, 121.86, 89.46, 56],
    "9": [16.8, 80+.5, 100-.5, 90, 122.22, 89.64, 57],
    "A": [24, 77-1, 92+5.5, 82.98, 124.2, 89.64, 56],
    "B": [27.72, 77-2.5, 95+1.5, 86.94, 127.44, 89.64, 56],
    "C": [30+1.5, 75-4, 95+1, 87.48, 126.18, 89.46, 56],
    "D": [25.56, 86-3.5, 98+2.5, 87.48, 122.04, 89.28, 56],
    "E": [28+1.5, 84-4, 98+2.5, 87.48, 121.5, 89.64, 56],
    "F": [33, 81.0-3.5, 98+1.5, 87.84, 121.14, 89.64, 56],
    "ABT": [27, 88, 103.34, 90, 117.9, 89.64, 56],
    "TRM": [30.5, 86-1, 103, 90, 117.9, 89.64, 56],
    "MEM": [28.62, 94, 105-1, 90.0, 117.72, 89.46, 56],
    "GO": [33, 92, 105.14, 90, 115.74, 89.46, 56],
}

# Programs to be sent to the MK14
programs = {
    "Duck Shoot":
        {
            "execAddr": "0F12",
            "hexLines": [
                ":180F1200C40D35C40031C401C8F4C410C8F1C400C8EEC40801C0E71EB2",
                ":180F2A00C8E49404C4619002C400C9808F01C0D89C0EC180E4FF980811",
                ":160F4200C8CEC0CAE480C8C64003FC0194D6B8BF98C8C40790CEDD"
            ]
        },
    "Moon Landing":
        {
            "execAddr": "0F52",
            "hexLines": [
                ":180F14000850009980009998000258003EC8E3C40135C8DFC40B31C877",
                ":180F2C00DBC0D702D40F01C180CF01C4008F04C0C91C1C1C1C010603EA",
                ":180F440094EDC400CF01C0BB35C0B93190CEC40F35C41431C40F36C4EA",
                ":180F5C002032C40CCAE4C10BCDFFBAE49CF8C40C37C4FF33C401CAE473",
                ":180F7400C5069404C5049032C402CAE302C5FFE902C900BAE39CF6C19A",
                ":180F8C00029402C499EDFFC900BAE494E3C50CAAE303C5FFF9FEC900A9",
                ":180FA40008BAE394F50694029004C400C9FFC1FF03EC94C9FDC499ECF9",
                ":180FBC0000C9FCC1003EC1F9940AC49903F9FA03EC009002C1FA3EC173",
                ":180FD400F73EC7FFC5F63EC40ACAE4C7FF940AE4DF9A31BAE49CF492E3",
                ":0A0FEC0049C109980333C90992496D"
            ]
        }
}

# Servo info
NUM_SERVOS = 7
isAllConverge = False
measuredForces = [0] * NUM_SERVOS
measuredRotationDegs = [0] * NUM_SERVOS

# Abort flag
globalAbortFlag = False

# Read data from 7bot - this is done in a separate thread and global variables are altered as the means
# of communication between the thread and the main program - ugly !
def botPortRead(ser):
    global isAllConverge, measuredRotationDegs, measuredForces, NUM_SERVOS
    global serialIsClosing
    rxBuf = [0] * (NUM_SERVOS * 2 + 1)
    beginFlag = False
    instruction = 0
    cnt = 0
    while True:
        # Handle closing down
        if serialIsClosing or not ser.isOpen():
            break
        # Get a char if there is one
        val = ser.read(1)
        if len(val) == 0:
            continue
        # print("Rx", "%02X " % ord(val))
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

# Utility functions
def appendTwoByteVal(buf, val):
    buf.append((int(val) // 128) & 0x7F)
    buf.append(int(val) & 0x7F)

def appendVecToSend(buf, vec):
    for el in vec:
        val = int(abs(el)) + (0 if el >= 0 else 1024)
        appendTwoByteVal(buf, val)

# Called while waiting for the robot arm to reach its destination
# Also allows the TKINTER UI to have some time to operate
def waitAndFlush(timeInSecs):
    for tii in range(int(timeInSecs * 1000)):
        sys.stdout.flush()
        masterTk.update_idletasks()
        masterTk.update()
        time.sleep(0.001)

# Limit value between two thresholds
def constrain(val, valMin, valMax):
    if val < valMin:
        return valMin
    if val > valMax:
        return valMax
    return val

# Set Servo angles
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
    #     bit 6: 0-disable ﬂuency, 1-enable ﬂuency;
    #     bit 5~0: speed value(range:0~25, 10 means 100 degrees per second)
    botPort.write(sendData)

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

# Move to a specific azimuth - the 7Bot standard firmware doesn't seem to do this in one go even though it
# can tell that the arm has not reached the desired position - not sure why this is but this is a fix which
# iterates towards the correct point by requesting positions greater or lesser than actually required until
# the arm gets near enough to the desired point
def moveToAzimuth(anglesDown):
    azimAngles = keyboardClearAngles[:]
    azimAngles[0] = anglesDown[0]
    implAngles = azimAngles[:]
    for i in range(5):
        setServoAngles(implAngles)
        while not isAllConverge and not globalAbortFlag:
            waitAndFlush(0.1)
        waitAndFlush(0.1)
        if globalAbortFlag:
            break
        angleErrs = calcAngleError(azimAngles)
        print("Azimuth attempt", i)
        dispAngleError(azimAngles)
        if abs(angleErrs[0]) < 1:
            break
        implAngles[0] += 1 if (angleErrs[0] > 0) else -1
    return implAngles[0]

# Send the robot arm to predefined locations
def goToHome():
    setServoAngles(homePositionAngles)
    while not isAllConverge and not globalAbortFlag:
        waitAndFlush(0.1)
    waitAndFlush(0.1)

def goToReady():
    setServoAngles(readyPositionAngles)
    while not isAllConverge and not globalAbortFlag:
        waitAndFlush(0.1)
    waitAndFlush(0.1)

def punchDownOnKey(armAngles, tryAdjustments):
    # Check for abort
    if globalAbortFlag:
        return
    print("  hovering     ", formatAngles(armAngles), "TryAdjust", tryAdjustments)
    # Initially go to a hover position above the key
    acceptedErrorDegs = [1, 1, 1, 1, 1]
    anglesToTry = armAngles[:]
    for i in range(len(keyPunchDownAngleDeltas)):
        anglesToTry[i] += keyPunchDownAngleDeltas[i]
    # Try several adjustments to get where we want to be
    setServoAngles(anglesToTry)
    while not isAllConverge and not globalAbortFlag:
        waitAndFlush(0.1)
    waitAndFlush(0.1)
    if globalAbortFlag:
        return
    print("  punchDown   ")
    setServoAngles(armAngles)
    while not isAllConverge and not globalAbortFlag:
        waitAndFlush(0.1)
    waitAndFlush(0.1)
    if globalAbortFlag:
        return
    dispAngleError(armAngles)
    print("  pullUp   ")
    setServoAngles(anglesToTry)
    while not isAllConverge and not globalAbortFlag:
        waitAndFlush(0.1)
    waitAndFlush(0.1)
    if globalAbortFlag:
        return

# Press a single key
def pressKey(key):
    print("pressKey", key, "...................")
    # Get the key position
    anglesDown = keyPositions[key]
    okAzimuth = moveToAzimuth(anglesDown)
    keyAngles = anglesDown[:]
    keyAngles[0] = okAzimuth
    punchDownOnKey(keyAngles, False)

# Send a sequence of keys (in a list)
def sendKeySequence(keys):
    for key in keys:
        if globalAbortFlag:
            return
        # Press the key
        pressKey(key)

# Set address on MK14
def setAddress(addr, forExec=False):
    print("Setting address to", addr)
    if forExec:
        keys = ["ABT", "GO"]
    else:
        keys = ["ABT", "MEM"]
    for ch in addr:
        keys.append(str(ch))
    sendKeySequence(keys)

# Send hex codes to the MK14
def sendHexCodes(hexcodes):
    digitPairs = [hexcodes[i:i + 2] for i in range(0, len(hexcodes), 2)]
    for pair in digitPairs:
        if globalAbortFlag:
            return
        keys = []
        keys.append("TRM")
        for ch in pair:
            keys.append(str(ch))
        keys.append("TRM")
        keys.append("MEM")
#        print(pair)
        sendKeySequence(keys)

# Convert a 2 digit hex number to decimal
def hexVal(inStr, pos, len):
    return int(inStr[pos:pos + len], 16)

# Sand an entire program to the MK14
def sendProgram(programName):
    if programName not in programs:
        return
    programDef = programs[programName]
    for hexLine in programDef["hexLines"]:
        leng = hexVal(hexLine, 1, 2)
        if leng > 0:
            setAddress(hexLine[3:7])
            sendHexCodes(hexLine[9:(9+leng*2)])
    setAddress(programDef["execAddr"], True)

# Debugging code
def formatAngles(angs):
    s = ""
    for i in range(len(angs)):
        s += "%6.1f" % (round(angs[i],1))
    return s

def dispAngleError(wantedAngles):
    angleErrs = calcAngleError(wantedAngles)
    print("            Wanted  ", formatAngles(wantedAngles))
    print("            Actual  ", formatAngles(measuredRotationDegs))
    print("            Errs    ", formatAngles(angleErrs))

def calcAngleError(wantedAngles):
    curAngles = measuredRotationDegs[:]
    angleErrs = []
    acceptedErrorDegs = [1, 1, 1, 1, 1]
    for angleIdx in range(len(acceptedErrorDegs)):
        angleErr = wantedAngles[angleIdx] - curAngles[angleIdx]
        angleErrs.append(round(angleErr,1))
    return angleErrs

# Function to handle UI abort key
def abortFn():
    global globalAbortFlag
    print("Aborted")
    globalAbortFlag = True

# Main program
print("Programming MK14 with 7Bot ...")
masterTk = tkinter.Tk()
masterTk.title("Programming MK14 with 7Bot")
masterTk.geometry("400x100")
cancelButton = tkinter.Button(masterTk, text="ABORT", command=abortFn)
cancelButton.pack(expand=tkinter.YES, fill=tkinter.BOTH)

# Serial connection to 7Bot
serialIsClosing = False
botPort = serial.Serial(port="COM12", baudrate=115200, timeout=1)
#botPort = serial.Serial(port="/dev/ttyAMA0", baudrate=115200, timeout=1)

# Thread for reading from port
thread = threading.Thread(target=botPortRead, args=(botPort,))
thread.start()
waitAndFlush(1)

# Set angles and wait for motion converge
print("Setting normal servo mode")
setForceStatus(1)
goToReady()

# Reboot 7Bot if previous status is not normal servo
# To make motion much more stable, highly recommend you use fluency all the time
print("Setting speed and fluency")
fluentEnables = [True, True, True, True, True, True, True]
setSpeed(fluentEnables, normalSpeeds)

# Set angles and wait for motion converge
print("Programming ...")

if len(testKeySequence) > 0:
    sendKeySequence(testKeySequence)
else:
    sendProgram(programToRun)

print("Pulling clear of the keyboard")
goToReady()

print("Going home")
goToHome()
setForceStatus(0)

serialIsClosing = True
waitAndFlush(1)
botPort.close()
