
import serial
import json
import time
from subprocess import Popen, PIPE
import os

DELIMETER = '#'
BAUD_RATE = 9600


class Coms:
    def __init__(self):
         # FIND ALL V5 DEVICES
         os.chdir("/dev/serial/by-id")
         process = Popen(['ls'], stdout=PIPE, stderr=PIPE)
         stdout, stderr = process.communicate()
         lines = stdout.decode('UTF-8').split('\n')
         if lines[0] == 'There are no connected VEX EDR V5 User ports\n':
             print("No brain connected")
             return

         self.brain_port = lines[-2]

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

comm = Coms()
while False:
    if comm.read() > 1:
        print("received")
        while comm.read() == 1:
            print("delay")
         
