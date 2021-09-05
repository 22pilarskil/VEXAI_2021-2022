
import serial
import json
ser = serial.Serial('/dev/cu.usbmodem1413', 9600, timeout=.1) 
import time
print(ser.name)
print(ser.isOpen())
while True:    
    #print(ser.readline().decode('ascii').strip('\n').strip('\r').split('#'))
    time.sleep(.1)
    ser.write(("header" + "#" + json.dumps("yo") + "\n").encode('ascii', 'replace'))


# import serial
# import json
# from subprocess import Popen, PIPE

# DELIMETER = '#'
# BAUD_RATE = 9600


# class Coms:
#     def __init__(self):
#         # FIND ALL V5 DEVICES
#         process = Popen(['prosv5', 'lsusb'], stdout=PIPE, stderr=PIPE)
#         stdout, stderr = process.communicate()
#         lines = stdout.decode('UTF-8').split('\n')

#         if lines[0] == 'There are no connected VEX EDR V5 User ports\n':
#             print("Plug in the fucking brain shitass")
#             return

#         port = lines[lines.index('VEX EDR V5 User ports:') + 1][
#                :lines[lines.index('VEX EDR V5 User ports:') + 1].index('-')].strip()
#         self.brain_port = port

#         self.ser = serial.Serial()
#         self.ser.baudrate = BAUD_RATE
#         self.ser.port = self.brain_port
#         self.ser.open()

#         print(self.ser.baudrate)
#         print(self.ser.port)

#         self.odom = 0, 0, 0

#     def __send(self, header, body):
#         self.ser.write((header + "#" + json.dumps(body) + "\n").encode('ascii', 'replace'))

#     def __read(self):
#         return json.loads(self.ser.readline().decode('ascii').strip('\n').strip('\r').split('#')[1])

#     def initSeq(self):
#         dic = self.__read().get("init", "uhoh")
#         if dic == "uhoh":
#             self.initSeq()
#         team = json.loads(dic).get("team", "uhoh")
#         if team == "uhoh":
#             self.initSeq()

#         if team == 'b':
#             return True
#         elif team == 'r':
#             return False



# comm = Coms()