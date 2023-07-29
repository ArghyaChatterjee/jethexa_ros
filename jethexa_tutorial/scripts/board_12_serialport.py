#!/usr/bin/python3
# Test the sending and receiving of the serial port. Run this program and then connect the tx and rx on the expansion board, and you can see the output "HELLO WORLD" in the command line
# When testing this program, the servo cannot be connected, because the data sent may be wrongly responded by the servo and cannot be processed normally
import serial
import time

if __name__ == "__main__":
    
    serialHandle = serial.Serial("/dev/ttyTHS1", 9600)
    while True:
        serialHandle.write(b"HELLO WORLD\r\n")
        time.sleep(0.1)
        count = serialHandle.in_waiting  # Get the data length of the serial port buffer
        print(count)
        if count != 0:  #If data is received
            recv = serialHandle.read(serialHandle.in_waiting)
            print(str(recv, encoding="utf8")) # print received data
        time.sleep(1)
    


