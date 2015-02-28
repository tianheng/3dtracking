import time
import serial
import sys
ser = serial.Serial(
    port='/dev/tty.usbmodem1421',
    baudrate=115200,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS
)


ser.isOpen()
out = ''
fo=open(sys.argv[1],"w")
out=''
while 1:
    try:
        out_temp=''
        while ser.inWaiting()>0:
            out_temp=ser.read(1)
            out+=out_temp
        if out_temp!='':
            sys.stdout.write(out_temp)
    except KeyboardInterrupt:
        fo.write(out)
        fo.close()
        raise

