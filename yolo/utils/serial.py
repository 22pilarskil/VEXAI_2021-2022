
import serial
import json
import os
from utils.decorators import exception

DELIMETER = '#'
BAUD_RATE = 9600

class Coms:
    def __init__(self):
         self.ser = serial.Serial(timeout=.1)
         self.ser.baudrate = BAUD_RATE
         self.open()

    @exception
    def open(self):
        os.chdir("/dev/serial/by-id")
        lines = os.listdir()
        for line in lines:
            if "if02" in line: self.ser.port = line
        self.ser.open()

    def send(self, header, body):
         print("SENDING {}".format(body))
         self.ser.write((header + "#" + json.dumps(body) + "\n").encode('ascii', 'replace'))

    def read(self, signals):
        data = {}
        msg = self.ser.readline().decode('ascii').strip('\n').strip('\r').split("#")
        for signal in signals:
            if signal in msg: data[signal] = msg[msg.index(signal) + 1]
        return data

    def wait(self, signal):
        while not self.read([signal])[signal]:
            print("Awaiting {} signal".format(signal))


