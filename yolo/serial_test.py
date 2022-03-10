from utils.serial import Coms

comm = Coms()
comm.open()
while True:
    x = comm.read(["continue", "mode"])
    if x: print(x)
