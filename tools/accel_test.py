import time
import sys
import math
fo=open(sys.argv[1],"r")
content=[]
speed=[]
angle=[]
flag=0
content=fo.readlines()
for each_line in content:
    if "***" in each_line:
        flag=0
    else:
        flag+=1
    if flag==2:
        #accel_lvl=math.floor((float(each_line)-300.00+32.00)/64)
        #speed.append(accel_lvl*64)
        #print accel_lvl*64
        speed.append(float(each_line)-300.00)

total_distance=0.00
prev_speed=0.00
for each_num in speed:
    prev_speed+=each_num*(1.00/100.00)*2.00*9.81/32768
    print total_distance
    total_distance+=prev_speed*(1.00/100.00)

print total_distance
