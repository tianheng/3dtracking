import time
import serial

ser = serial.Serial(
    port='/dev/cu.usbmodem1411',
    baudrate=9600,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS
)


ser.isOpen()
out = ''
flag=100
fo=open("data.txt","w")
while flag:
    while ser.inWaiting()>0:
        out+=ser.read(1)
    if out!='':
        print out
        fo.write(out)
    flag-=1
fo.close()