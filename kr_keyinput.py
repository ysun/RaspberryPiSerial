#!/usr/bin/env python
#coding: utf-8
from evdev import InputDevice
from select import select

import sys, getopt
import os

import time
import subprocess

import multiprocessing

key_code = 0
key_value = 0

opts, args = getopt.getopt(sys.argv[1:], "hi:o:")
#input_file=""
#output_file=""
# 
#for op, value in opts:
#	if op == "-i":
#		input_file = value
#	elif op == "-o":
#		output_file = value
#	elif op == "-h":
#		usage()
#		sys.exit()

dirList = []
fileList = []
processList = []
g_distance = 100

def printPath(path):
    global fileList
    files = os.listdir(path)
    for f in files:
#        print f[0:6]
        if f[0:5] == 'event':
            fileList.append(f)
            print "new device: %s"%f


if len(sys.argv) > 1:
    device_input = sys.argv[1]
    fileList.append("/dev/input/%s"%device_input)
else:
    printPath('/dev/input')
    
def detectInputKey(fname):
    device_input = "/dev/input/%s"%fname
    print "listening: %s" %device_input
    dev = InputDevice(device_input)
    checksum = 0
    global g_distance;

    while True:
        select([dev], [], [])
        for event in dev.read():
            checksum = (checksum + 1)%2
            key_code = event.code
            key_value = event.value

            print "code:%s value:%s" % (event.code, event.value)
            if key_code == 5:
               if key_value >= 0 and key_value < 1:
                   print "move up"
               elif key_value > 254 and key_value <= 255:
                   print "move down"

            if key_code == 2:
               if key_value >= 0 and key_value < 1:
                   print "move left"
                   os.system("./control -t 300,B,0,-%d,0" %g_distance);
               elif key_value > 254 and key_value <= 255:
                   print "move right"
                   os.system("./control -t 300,B,0,%d,0"%g_distance);

            if key_code == 304:
               if key_value >= 1:
                   print "speed up"
                   os.system("./control -l v,-5");

            if key_code == 305:
               if key_value >= 1:
                   print "speed down"
                   os.system("./control -l v,5");

            if key_code == 307:
               if key_value >= 1:
                   print "distance longer: now %d" %g_distance
                   g_distance += 5
                   if g_distance > 1000:
                       g_distance = 1000

            if key_code == 308:
               if key_value >= 1:
                   print "distance shorter: now %d" %g_distance
                   g_distance -= 5
                   if g_distance < 0:
                       g_distance = 0


            #print "code:%s value:%s" % (event.code, event.value)

#           if key_code == 1 and key_value == 0 and checksum:
#               print "move up"
#
#            elif key_code == 1 and key_value == 255 and checksum:
 #               print "move down"
 #
  #          elif key_code == 1 and key_value == 128 and checksum:
  #              print "N/A"
#
#            elif key_code == 0 and key_value == 0 and checksum:
 #               print "move left"
 #
  #          elif key_code == 0 and key_value == 128 and checksum:
  #              print "N/A"
#
#            elif key_code == 0 and key_value == 255 and checksum:
 #               print "move right"
 #
if __name__ == '__main__':
#    printPath('/dev/input')
#  subprocess.Popen(['/bin/sh', '-c', 'cd /usr/lib/edison_config_tools/blockr/mjpg-streamer && ./start.sh'])
  while 1:
    for fname in fileList:
        print fname

        p = multiprocessing.Process( target=detectInputKey, args=(fname, ))
        p.start()
        processList.append(p)


#    printPath('/dev/input')
#    time.sleep(10000)

    for p in processList:
        p.join()

#    detectInputKey()
