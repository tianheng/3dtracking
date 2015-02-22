import time
import sys

fo=open("data.txt","r")
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
    if flag==5:
        speed.append(float(each_line))
        print float(each_line)

prev_angle=0.00
for each_num in speed:
    new_angle=prev_angle+each_num*1.00/100.00*250.00/32768.00
    prev_angle=new_angle
    angle.append(new_angle)

fout=open("gyro.txt","w")
for each_num in angle:
    fout.write(str(each_num)+'\n')
fo.close()
fout.close()
