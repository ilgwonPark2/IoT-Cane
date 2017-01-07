# test BLE Scanning software
# jcs 6/8/2014
import MySQLdb
import blescan
import sys
import bluetooth._bluetooth as bluez
import time
import os
import subprocess
import RPi.GPIO as gpio
import time
from multiprocessing import Process, Lock


gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
trigTop = 13
echoTop = 19
trigBottomM = 20
echoBottomM = 21
trigBottomR = 5
echoBottomR = 6
trigBottomL = 17
echoBottomL = 27
pin1 = 2
pin2 = 3
pin3 = 8
pin4 = 7
gpio.setup(trigTop, gpio.OUT)
gpio.setup(trigBottomM, gpio.OUT)
gpio.setup(trigBottomR, gpio.OUT)
gpio.setup(trigBottomL, gpio.OUT)
gpio.setup(echoTop, gpio.IN)
gpio.setup(echoBottomM, gpio.IN)
gpio.setup(echoBottomR, gpio.IN)
gpio.setup(echoBottomL, gpio.IN)
gpio.setup(pin1, gpio.OUT)
gpio.setup(pin2, gpio.OUT)
gpio.setup(pin3, gpio.OUT)
gpio.setup(pin4, gpio.OUT)

lock=Lock()

def motorBoth() :
    gpio.output(pin1, True)
    gpio.output(pin2, False)
    gpio.output(pin3, True)
    gpio.output(pin4, False)

def motorLeft():
    gpio.output(pin1, True)
    gpio.output(pin2, False)
    gpio.output(pin2, True)
    gpio.output(pin3, True)

def motorRight():
    gpio.output(pin1, True)
    gpio.output(pin2, True)
    gpio.output(pin3, True)
    gpio.output(pin4, False)

def motorZero() :
    gpio.output(pin1, True)
    gpio.output(pin2, True)
    gpio.output(pin3, True)
    gpio.output(pin4, True)

def calculateD(txpower, rssi) :
    if(rssi == 0) :
        return -1.0
    ratio = rssi*1.0/txpower
    if ratio < 1.0 :
        return ratio**10
    else :
        distance = 0.89976*(ratio**7.7095) + 0.111
        return distance

# method to use tts
def say(words):
    lock.acquire()
    tempfile = "temp.wav"
    devnull = open("/dev/null","w")
    subprocess.call(["pico2wave", "-w", tempfile, words],stderr=devnull)
    subprocess.call(["aplay", tempfile],stderr=devnull)
    lock.release()
# os.remove(tempfile)

def Beacon():
    while True :
        returnedList = blescan.parse_events(sock, 10)
        print "----------"
        minDistance = 100
        minID = " "
        for beaconid in returnedList:
            beaconarr= beaconid.split(',')
            txPower = beaconarr[4]
            txPowerint = int(txPower)
            rssi=beaconarr[5]
            rssiint = int(rssi)
            distance = calculateD(txPowerint,rssiint)
            if distance < minDistance :
                minID = beaconid
                minDistance = distance

        for beaconid2 in returnedList:
            for row in result :
                 if row[0] == beaconid2[0:17] and beaconid2[0:17] == minID[0:17] :
                    MAC_ADDRESS = row[0]
                    Location = row[2]
                    Severity = row[3]
                    beaconarr = beaconid2.split(',')
                    txPower = beaconarr[4]
                    txPowerint = int(txPower)
                    rssi = beaconarr[5]
                    rssiint = int(rssi)
                    distance = calculateD(txPowerint,rssiint)
                    distanceTemp = round(distance, 2)
                    distanceStr = str(distanceTemp)
                    print distance
                    print str(MAC_ADDRESS) +", "+str(Location) + "," + str(distanceStr)
                    distanceVoice = distanceStr +"meters  left"

                    if distance < 2 :
                        if Severity == 1 :
                            motorRight()
                            print("Dangerous There is an obstacle")
                            say("Dangerous There is an obstacle")
                            time.sleep(3)
                        else :
                            motorZero()

                        if len(sys.argv)<=1:
                            say(Location)
                            say(distanceVoice)
                            time.sleep(1)
		`	   continue
                        elif os.path.isfile(sys.argv[1]):
                            fi=open(sys.argv[1],'r')
                            text=fi.read()
                            say(text)
                        else:
                            sentence=""
                            for i in range(1,len(sys.argv)):
                                sentence=sentence+sys.argv[i]
                            say(sentence)


def Ultra():
    while True :
        gpio.output(trigTop, False)
        time.sleep(0.02)

        gpio.output(trigTop, True)
        time.sleep(0.1)
        gpio.output(trigTop, False)

        while gpio.input(echoTop) == 0 :
            global pulse_startTop
            pulse_startTop = time.time()

        while gpio.input(echoTop) == 1 :
            pulse_endTop = time.time()

        gpio.output(trigBottomM, False)
        time.sleep(0.02)

        gpio.output(trigBottomM, True)
        time.sleep(0.1)
        gpio.output(trigBottomM, False)

        while gpio.input(echoBottomM) == 0 :
            pulse_startBottomM = time.time()

        while gpio.input(echoBottomM) == 1 :
            pulse_endBottomM = time.time()

        gpio.output(trigBottomR, False)
        time.sleep(0.02)

        gpio.output(trigBottomR, True)
        time.sleep(0.1)
        gpio.output(trigBottomR, False)

        while gpio.input(echoBottomR) == 0 :
            pulse_startBottomR = time.time()

        while gpio.input(echoBottomR) == 1 :
            pulse_endBottomR = time.time()

        gpio.output(trigBottomL, False)
        time.sleep(0.02)

        gpio.output(trigBottomL, True)
        time.sleep(0.1)
        gpio.output(trigBottomL, False)

        while gpio.input(echoBottomL) == 0 :
            pulse_startBottomL = time.time()

        while gpio.input(echoBottomL) == 1 :
            pulse_endBottomL = time.time()

        pulse_durationTop = pulse_endTop - pulse_startTop
        distanceTop = pulse_durationTop * 17000
        distanceTop = round(distanceTop, 2)

        pulse_durationBottomM = pulse_endBottomM - pulse_startBottomM
        distanceBottomM = pulse_durationBottomM * 17000
        distanceBottomM = round(distanceBottomM, 2)

        pulse_durationBottomR = pulse_endBottomR - pulse_startBottomR
        distanceBottomR = pulse_durationBottomR * 17000
        distanceBottomR = round(distanceBottomR, 2)

        pulse_durationBottomL = pulse_endBottomL - pulse_startBottomL
        distanceBottomL = pulse_durationBottomL * 17000
        distanceBottomL = round(distanceBottomL, 2)


        print "DistanceTop : ", distanceTop, "cm"
        print "DistanceBottomM : ", distanceBottomM, "cm"
        print "DistanceBottomR : ", distanceBottomR, "cm"
        print "DistanceBottomL : ", distanceBottomL, "cm"


        if(distanceBottomR < 100) :
            motorRight()
            sentenceR = "watch out right side"
            say(sentenceR)
            print(sentenceR)
            time.sleep(1)
        else :
            motorZero()

        if(distanceBottomL < 100) :
            motorRight()
            sentenceL="watch out Left Side"
            say(sentenceL)
            print(sentenceL)
            time.sleep(1)
        else :
            motorZero()

        if(distanceBottomM<=100) :
            if (distanceTop-distanceBottomM)<50 :
                motorBoth()
                sentenceBarrier = "it seems there is a wall"
                say(sentenceBarrier)
                time.sleep(1)
            else :
                motorRight()
        time.sleep(5)

dev_id = 0
try:
    sock = bluez.hci_open_dev(dev_id)
    print "ble thread started"

except:
    print "error accessing bluetooth device..."
    sys.exit(1)

blescan.hci_le_set_scan_parameters(sock)
blescan.hci_enable_le_scan(sock)

# Open database connection
db=MySQLdb.connect("localhost","root","1234","beacondb")
# prepare a cursor object using cursor() method
cursor = db.cursor()
sql = "SELECT * FROM info"
cursor.execute(sql)
result = cursor.fetchall()

try:
    motorZero()
    a = Process(target=Beacon)
    b = Process(target=Ultra)
    a.start()
    b.start()
   # th1 = Thread(target=Beacon)
   # th2 = Thread(target=Ultra)
   # th1.start()
   # th2.start()
except:
    gpio.cleanup()
    motorZero()