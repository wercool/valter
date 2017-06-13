#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
from subprocess import Popen, PIPE
from time import sleep
import os
import sys
import Eliza

def main():
    print "Valter Talks"
    if len(sys.argv) > 1:
        if sys.argv[1] == "chat":
            print "Chat Mode"
            while True:
                statement = raw_input("> ")
                print Eliza.analyze(statement)
                if statement == "quit":
                    break
    else:
        googleSpeechRecognition()

def googleSpeechRecognition():
    pipe = Popen(["espeak", "-vru", "-s130", "Подгот овка системы распознав ания русской речи"], stdout=PIPE)
    pipe = Popen(["chromium-browser", "--start-maximized", "https://translate.google.com/#ru/en"], stdout=PIPE)
    sleep(10.0)
    pipe = Popen(["espeak", "-vru", "-s130", "Я гот ов"], stdout=PIPE)
    pipe.communicate()
    while True:
        GoogleResult = ""

        pipe = Popen(["aplay", "sounds/beep1.wav"], stdout=PIPE)
        pipe.communicate()
        os.system("xdotool mousemove --sync 2020 411 click 1")                              #start/stop voice recognition
        sleep(0.5)
        os.system("xdotool mousemove --sync 0 0")                                           #move mouse out

        sleep(5.0)

        os.system("xdotool key --delay 0 --clearmodifiers ctrl+a")                          #ctrl+a
        sleep(0.5)
        os.system("xdotool key --delay 0 --clearmodifiers ctrl+c")                          #ctrl+с
        sleep(0.5)
        os.system("xdotool mousemove --sync 2621 317 click 1")                              #clear google input filed
        sleep(0.5)
        os.system("xdotool mousemove --sync 2020 411 click 1")                              #start/stop voice recognition
        sleep(0.5)
        os.system("xdotool mousemove --sync 0 0")                                           #move mouse out

        pipe = Popen(["xclip", "-o"], stdout=PIPE)
        GoogleResult = pipe.communicate()[0]

        Popen(["xclip", "-i", "/dev/null"], stdout=PIPE)

        if GoogleResult:
            GoogleResult = GoogleResult.decode('utf-8').lower()
            GoogleResult = GoogleResult.encode('utf-8')
            print "GoogleResult:" + GoogleResult
            ElizaAnswer = Eliza.analyze(GoogleResult)
            print "ElizaAnswer:" + ElizaAnswer
            if ElizaAnswer:
                pipe = Popen(["espeak", "-vru", "-s130", ElizaAnswer], stdout=PIPE)
                pipe.communicate()

if __name__ == "__main__":
    main()
