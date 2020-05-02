#!/usr/bin/env python3

#Owen Gibson, Brian Rupp, Ryan Miller
#Dr. Gregory Lee
#EECS 398/399
#3D Printed Biomimetic Robotic Finger

import serial
import crcmod
import struct
import threading
import time
import paho.mqtt.client as mqtt
import json
import pandas as pd
import xlrd as xl

from finger_common import *

#MQTT Setup
MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883
MQTT_BROKER_KEEP_ALIVE_SECS = 60
MQTT_CLIENT_ID = "finger_robot"

#Device Setup
deviceName = '/dev/tty.usbserial-AI027ZS0'  #changes depending on laptop connection
baudRate = 50000
serialPort = serial.Serial (deviceName, baudrate=baudRate, timeout=0.05) #change baudrate
lock = threading.Lock() #Mutux for memory allocation

class InvalidFingerConfig(Exception):
    pass

class MotorControl(object):

    _motorID = 2

    def _get_motorID(self):
        return self._motorID

    def _set_motorID(self, value):
        self._motorID = value

    def __init__(self):
        self._client = self._create_and_configure_broker_client()

    def calculateCheckSum(self, array):
        totalHexValue = 0
        for hexValue in array:
            totalHexValue = totalHexValue + hexValue

        truncatedHex = "0x"
        if totalHexValue > 255:
            for i in (3,4):
                truncatedHex = truncatedHex + hex(totalHexValue)[i]
            return 255 - int(truncatedHex, 16)

        checkSumValue = 255 - totalHexValue
        return checkSumValue

    def goalPosition(self, input):
        writeInstruction = 0x03
        writePacketLength = 0x05
        goalPositionAddress = 0x1e
        goal = "0x0"
        goal2 = "0x"

        setGoal = int(input)
        maxPosition = self.readExcel('Max Position')
        if setGoal > maxPosition:
            setGoal = maxPosition

        if setGoal >= 0 & setGoal <= 4095:
            formatInput = "{0:#0{1}x}".format(setGoal,6)
            goal = goal + formatInput[3]
            goal2 = goal2 + formatInput[4] + formatInput[5]

            goal = int(goal, 16)
            goal2 = int(goal2, 16)
        else:
            print("ERROR: Your range is invalid.")

        goalPositionArray = [self._motorID, writePacketLength, writeInstruction, goalPositionAddress, goal, goal2]
        goalPositionCheck = self.calculateCheckSum(goalPositionArray)
        goalPositionPacket = bytearray([0xff, 0xff, self._motorID, writePacketLength, writeInstruction, goalPositionAddress, goal2, goal, goalPositionCheck])
        crcFun = crcmod.mkCrcFun(0x18005, initCrc=0, rev=False)
        crc = crcFun(bytes(goalPositionPacket))
        goalPositionPacket.extend(struct.pack('<H', crc))
        try:
            serialPort.write(goalPositionPacket)
            print("Goal Position set to " + str(input))
        finally:
            lock.release()

    def torqueLimit(self):
        writeInstruction = 0x03
        writePacketLength = 0x05
        limitAddress = 0x22
        toggleLimit = self.readExcel('Torque Limit')
        maxTorque = 0x98
        calibrationTorque = 0x33
        limitCheckArray = [self._motorID, writePacketLength, writeInstruction, limitAddress, 0x00, toggleLimit]
        limitCheck = self.calculateCheckSum(limitCheckArray)

        limitPacket = bytearray([0xff, 0xff, self._motorID, writePacketLength, writeInstruction, limitAddress, toggleLimit, 0x00, limitCheck])
        crcFun = crcmod.mkCrcFun(0x18005, initCrc=0, rev=False)
        crc = crcFun(bytes(limitPacket))
        limitPacket.extend(struct.pack('<H', crc))

        try:
            serialPort.write(limitPacket)
            print("Torque Limit Set")
        finally:
            lock.release()

    def readPresentSpeed(self):
        readInstruction = 0x02
        readPacketLength = 0x04
        presentSpeedAddress = 0x26
        dataLength = 0x01
        readMovingArray = [self._motorID, readPacketLength, readInstruction, presentSpeedAddress, dataLength]
        movingCheck = self.calculateCheckSum(readMovingArray)

        movingPacket = bytearray([0xff, 0xff, self._motorID, readPacketLength, readInstruction, presentSpeedAddress, dataLength, movingCheck])
        crcFun = crcmod.mkCrcFun(0x18005, initCrc=0, rev=False)
        crc = crcFun(bytes(movingPacket))
        movingPacket.extend(struct.pack('<H', crc))

        try:
            serialPort.write(movingPacket)
            print("Reading Present Speed...")
        finally:
            lock.release()

        lock.acquire()
        try:
            header = serialPort.read(5) #read 3 bytes
            speed = serialPort.read(1)
            check = serialPort.read(1)
            print("Type:" + str(type(speed)))
            #speedInt = int(":".join("{:02x}".format(ord(c)) for c in speed), 16)
            print('Received header: ' + ":".join("{:02x}".format(ord(c)) for c in header))
            print('Received checksum: ' + ":".join("{:02x}".format(ord(c)) for c in check))
            print(speedInt)
            return speedInt
        finally:
            lock.release()

    def readPresentPosition(self):
        readInstruction = 0x02
        readPacketLength = 0x04
        presentPositionAddress = 0x24
        dataLength = 0x02
        readPositionArray = [self._motorID, readPacketLength, readInstruction, presentPositionAddress, dataLength]
        positionCheck = calculateCheckSum(readPositionArray)

        positionPacket = bytearray([0xff, 0xff, self._motorID, readPacketLength, readInstruction, presentPositionAddress, dataLength, positionCheck])
        crcFun = crcmod.mkCrcFun(0x18005, initCrc=0, rev=False)
        crc = crcFun(bytes(positionPacket))
        positionPacket.extend(struct.pack('<H', crc))

        try:
            serialPort.write(positionPacket)
            print("Reading Present Position...")
        finally:
            lock.release()

        lock.acquire()
        try:
            header = serialPort.read(5) #read 3 bytes
            position = serialPort.read(2)
            check = serialPort.read(1)
            print('Received header: ' + ":".join("{:02x}".format(ord(c)) for c in header))
            print('Received position: ' + ":".join("{:02x}".format(ord(c)) for c in position))
            print('Received checksum: ' + ":".join("{:02x}".format(ord(c)) for c in check))
        finally:
            lock.release()

    def angleLimit(self):
        writeInstruction = 0x03
        writePacketLength = 0x05
        angleLimitAddress = 0x08
        limit1 = 0x00
        limit2 = 0x00
        limitCheckArray = [self._motorID, writePacketLength, writeInstruction, angleLimitAddress, limit1, limit2]
        limitCheck = calculateCheckSum(limitCheckArray)

        limitPacket = bytearray([0xff, 0xff, self._motorID, writePacketLength, writeInstruction, angleLimitAddress, limit2, limit1, limitCheck])
        crcFun = crcmod.mkCrcFun(0x18005, initCrc=0, rev=False)
        crc = crcFun(bytes(limitPacket))
        limitPacket.extend(struct.pack('<H', crc))

        try:
            serialPort.write(limitPacket)
            print("Angle Limit Set")
        finally:
            lock.release()

    def toggleTorque(self):
        # lock.acquire()
        writeInstruction = 0x03
        writePacketLength = 0x04
        torqueAddress = 0x18
        torqueOn = 0x01
        torqueOff = 0x00
        torqueCheckArray = [self._motorID, writePacketLength, writeInstruction, torqueAddress, torqueOff]

        torqueCheck = calculateCheckSum(torqueCheckArray)
        torquePacket = bytearray([0xff, 0xff, self._motorID, writePacketLength, writeInstruction, torqueAddress, torqueOff, torqueCheck])
        crcFun = crcmod.mkCrcFun(0x18005, initCrc=0, rev=False)
        crc = crcFun(bytes(torquePacket))
        torquePacket.extend(struct.pack('<H', crc))
        try:
            serialPort.write(torquePacket)
            print("Torque Disabled")
        finally:
            lock.release()

    def calibration(self):

        print("Entered Calibration")

        #Return to home position
        lock.acquire()
        self.goalPosition(0)
        time.sleep(1)

        #Wait until at home position
        lock.acquire()
        while self.readPresentSpeed() != 0:
            lock.acquire()
            print("Still moving...")
            time.sleep(1)
        print ("Reached Goal 0")

        #Set torque low
        lock.acquire()
        self.torqueLimit(0x33)
        time.sleep(1)

        #Set extreme goal position
        lock.acquire()
        self.goalPosition(3000)
        time.sleep(1)

        #Wait until stop moving
        lock.acquire()
        while self.readPresentSpeed() != 0:
            lock.acquire()
            print("Still moving...")
            time.sleep(1)

        #Read current position where stopped

        #Write current position as CCW angle limit

        #Set torque high
        print ("Stopped Moving")
        time.sleep(1)
        lock.acquire()
        self.torqueLimit(0x98)
        time.sleep(1)

        #set goal position to first extreme
        #if motor stops moving, set that as one range
        #set goal position to other extreme
        #if motor stops moving, set that as other range

    def setStatusLevel(self):
        writeInstruction = 0x03
        writePacketLength = 0x04
        statusAddress = 0x10
        level = 0x01

        statusLevelArray = [self._motorID, writePacketLength, writeInstruction, statusAddress, level]

        statusCheck = calculateCheckSum(statusLevelArray)
        statusPacket = bytearray([0xff, 0xff, self._motorID, writePacketLength, writeInstruction, statusAddress, level, statusCheck])
        crcFun = crcmod.mkCrcFun(0x18005, initCrc=0, rev=False)
        crc = crcFun(bytes(statusPacket))
        statusPacket.extend(struct.pack('<H', crc))
        try:
            serialPort.write(statusPacket)
            print("Status Level Changed")
        finally:
            lock.release()

    def _create_and_configure_broker_client(self):
        client = mqtt.Client(client_id=MQTT_CLIENT_ID, protocol=MQTT_VERSION)
        client.message_callback_add(TOPIC_SET_FINGER_CONFIG,
                                    self.on_message_set_config)
        client.on_connect = self.on_connect
        client.on_message = self.default_on_message
        return client

    def on_message_set_config(self, client, userdata, msg):
        try:
            new_config = json.loads(msg.payload)
            # if 'motorID' in new_config:
            #     self._motorID = new_config['motorID']
            #     print (type(self._motorID))
            if 'motorID' in new_config:
                self._motorID = new_config['motorID']
            if 'goalPosition' in new_config:
                positionSet = new_config['goalPosition']
                lock.acquire()
                self.goalPosition(positionSet)

        except InvalidFingerConfig:
            print("error applying new settings " + str(msg.payload))

    def default_on_message(self, client, userdata, msg):
        print("Received message on topic " +
              msg.topic + " with payload '" + str(msg.payload) + "'")

    def serve(self):
        self._client.connect(MQTT_BROKER_HOST,
                             port=MQTT_BROKER_PORT,
                             keepalive=MQTT_BROKER_KEEP_ALIVE_SECS)
        self._client.loop_forever()

    def on_connect(self, client, userdata, rc, unknown):
        self._client.publish(client_state_topic(MQTT_CLIENT_ID), "1",
                             qos=2, retain=True)
        self._client.subscribe(TOPIC_SET_FINGER_CONFIG, qos=1)

    def runGoal(self):
        lock.acquire()
        # presentPositionThread = threading.Thread(target=readPresentPosition)
        # presentPositionThread.start()

        # presentSpeedThread = threading.Thread(target=readPresentSpeed)
        # presentSpeedThread.start()
        # calibrationThread = threading.Thread(target=calibration)
        # calibrationThread.start()
        print("Enter a number from 0 - 4095:")
        global input
        input = input()
        self.goalPosition(input)
        # goalPositionThread = threading.Thread(target=goalPosition, args=(input,))
        # goalPositionThread.start()

    def readExcel(self, column):
        excel_data = pd.read_excel("roboticConfig.xlsx", sheet_name = "Sheet1")
        return excel_data[column][0]

if __name__ == "__main__":
    lock.acquire()
    torque = MotorControl().torqueLimit()
    finger = MotorControl().serve()
