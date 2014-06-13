import socket
import serial
from time import sleep
import time
import threading

MAXLINE = 100

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)

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

def threaded_ping():
    while True:
        ser.write("PING")
#        print "PING"
        sleep(0.5)

def threaded_reader():
    while True:
        reader_result = ser.readline().strip()
        if not(conn is None):
            if (reader_result):
                print ">" + reader_result
                conn.send(reader_result + '\n')
                reader_result = None
        sleep(0.001)

ping = threading.Thread(target=threaded_ping, args = ())
ping.daemon = True
ping.start()

HOST = ''                
PORT = 9001
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn = None

reader = threading.Thread(target=threaded_reader, args = ())
reader.daemon = True
reader.start()

while True:
    print "started..."
    conn, addr = s.accept()
    print 'Connected by', addr
    for line, err in linesplit(conn, MAXLINE):
        if err:
            print "Error: Line greater than allowed length %d (got %d)"  % (MAXLINE, len(line))
            break
        else:
            cmd = line.strip()
            ser.write(cmd)
            cur_time = int(round(time.time() * 1000))
            print "< [", cur_time, "]", cmd

conn.close()
