
import serial
import json
import os

DELIMETER = '#'
BAUD_RATE = 9600

class Coms:
    def __init__(self):
         self.ser = serial.Serial(timeout=.1)
         self.ser.baudrate = BAUD_RATE

    def send(self, header, body):
         self.ser.write((header + "#" + json.dumps(body) + "\n").encode('ascii', 'replace'))

    def open(self):
        os.chdir("/dev/serial/by-id")
        lines = os.listdir()
        for line in lines:
            if "if02" in line: self.ser.port = line
        self.ser.open()
    
    def read(self, signal):
        msg = self.ser.readline().decode('ascii').strip('\n').strip('\r').split("#")
        print(msg)
        if signal in msg: return True
        return False


