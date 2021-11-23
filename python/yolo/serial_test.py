
import serial
import json
import time
from subprocess import Popen, PIPE
import os
import gc

DELIMETER = '#'
BAUD_RATE = 9600

class Coms:
    def __init__(self):
         # FIND ALL V5 DEVICES
         os.chdir("/dev/serial/by-id")
         lines = os.listdir()
         for line in lines:
             if "if02" in line: self.brain_port = line

         self.ser = serial.Serial(timeout=.1)
         self.ser.baudrate = BAUD_RATE
         self.ser.port = self.brain_port
         self.ser.open()

         print(self.ser.baudrate)
         print(self.ser.port)


    def send(self, header, body):
         self.ser.write((header + "#" + json.dumps(body) + "\n").encode('ascii', 'replace'))

    def read(self):
        return len(self.ser.readline().decode('ascii').strip('\n').strip('\r').split("#"))
        #return json.loads(self.ser.readline().decode('ascii').strip('\n').strip('\r').split('#')[1])


