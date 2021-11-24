
import serial
import json
import os

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

    def read(self, signal):
        body = self.ser.readline().decode('ascii').strip('\n').strip('\r').split("#")
        if signal in body: 
            return True


