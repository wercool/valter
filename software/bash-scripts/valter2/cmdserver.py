#!/usr/bin/env python

import socket
import serial
from time import sleep
import time
import threading
import os

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

HOST = ''
PORT = 10002
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn = None

while True:
    print "commands server ready..."
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
            print "<", cmd
conn.close()
