
import serial
import json
import os
from utils.decorators import exception, bcolors

DELIMETER = '#'
BAUD_RATE = 9600

class Coms:
    def __init__(self):
         self.ser = serial.Serial(timeout=.1)
         self.ser.baudrate = BAUD_RATE
         self.open()
         self.data = {}

    @exception
    def open(self):
        os.chdir("/dev/serial/by-id")
        lines = os.listdir()
        for line in lines:
            if "if02" in line: self.ser.port = line
        self.ser.open()

    def send(self, header, body):
         bcolors.print("SENDING {}".format(body),"cyan")
         self.ser.write((header + "#" + json.dumps(body) + "\n").encode('ascii', 'replace'))

    def read(self, signals):
        
        msg = self.ser.readline().decode('ascii').strip('\n').strip('\r').split("#") 
        print("MSG: " + str(msg))
        for signal in signals:
            if signal in msg: 
                self.data[signal] = msg[msg.index(signal) + 1]
        data = self.data
        print("DATA: " + str(data))
        
        if not "@" in msg: 
            self.data = {}
            
        return data

    def wait(self, signal):
        signal+=["camera"]
        print("Awaiting {} signal".format(signal))
        while not self.read(signal):
            pass


