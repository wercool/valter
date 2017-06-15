#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
from subprocess import Popen, PIPE
from time import sleep
from time import gmtime, strftime
import os
import re
import sys
from wikipedia import Wikipedia
from wiki2plain import Wiki2Plain
import Eliza

lang = 'ru'
wiki = Wikipedia(lang)

def sayToEliza(statement, inVoice):
    ElizaAnswer = Eliza.analyze(statement)
    if ElizaAnswer:
        if ElizaAnswer == "time":
            hours = strftime("%H", gmtime())
            minutes = strftime("%M", gmtime())
            ElizaAnswer = "Сейчас " + hours + " часов " + minutes + " минут"
        if ElizaAnswer.find("wiki:") > -1:
            try:
                raw = wiki.article(ElizaAnswer[4:])
            except:
                raw = None
            if raw:
                wiki2plain = Wiki2Plain(raw)
                content = wiki2plain.text
#                print content
                content_parts = content.split("\n")
                paragraph_index = 0
                for p in range(0, 3):
                    paragraph = content_parts[p].decode("utf-8")
                    if (paragraph.find('|') > -1) or (paragraph.find("NOTOC") > -1):
                        paragraph_index = 2
                content = content_parts[paragraph_index].decode("utf-8")
                first_dot_index = content.find('.')
                ElizaAnswer = content[0: first_dot_index]
                if (len(ElizaAnswer) < 50):
                    second_dot_index = content[first_dot_index + 1: ].find('.')
                    ElizaAnswer = content[0: second_dot_index]
                ElizaAnswer = re.sub(r'[!@#$=*]', '', ElizaAnswer)

        print "> " + remove_tags(ElizaAnswer)

        if inVoice:
            pipe = Popen(["espeak", "-vru", "-s100", "-p10", "-m", ElizaAnswer], stdout=PIPE)
            pipe.communicate()

def remove_tags(s):
    tag = False
    quote = False
    out = ""

    for c in s:
            if c == '<' and not quote:
                tag = True
            elif c == '>' and not quote:
                tag = False
            elif (c == '"' or c == "'") and tag:
                quote = not quote
            elif not tag:
                out = out + c

    return out

def main():
    print "Valter Talks"
    if len(sys.argv) > 1:
        if sys.argv[1] == "chat":
            print "Chat Mode"
            while True:
                statement = raw_input("< ")
                sayToEliza(statement, False)
        elif sys.argv[1] == "ttschat":
            while True:
                statement = raw_input("< ")
                sayToEliza(statement, True)
    else:
        googleSpeechRecognition()

def googleSpeechRecognition():

    #laboratory
    """
    startStopVoiceRecognitionX = 2020
    startStopVoiceRecognitionY = 411
    clearGoogleInputFiledX = 2621
    clearGoogleInputFiledY = 317
    """
    #office
    startStopVoiceRecognitionX = 1373
    startStopVoiceRecognitionY = 376
    clearGoogleInputFiledX = 1973
    clearGoogleInputFiledY = 279

    pipe = Popen(["espeak", "-vru", "-s130", "Подгот овка системы распознав ания русской речи"], stdout=PIPE)
    pipe = Popen(["chromium-browser", "--start-maximized", "https://translate.google.com/#ru/en"], stdout=PIPE)
    sleep(10.0)
    pipe = Popen(["espeak", "-vru", "-s130", "Я гот ов"], stdout=PIPE)
    pipe.communicate()
    while True:
        GoogleResult = ""

        pipe = Popen(["aplay", "sounds/beep1.wav"], stdout=PIPE)
        pipe.communicate()
        os.system("xdotool mousemove --sync %d %d click 1" % (startStopVoiceRecognitionX, startStopVoiceRecognitionY))          #start/stop voice recognition
        sleep(0.5)
        os.system("xdotool mousemove --sync 0 0")                                                                               #move mouse out

        sleep(5.0)

        os.system("xdotool key --delay 0 --clearmodifiers ctrl+a")                                                              #ctrl+a
        sleep(0.5)
        os.system("xdotool key --delay 0 --clearmodifiers ctrl+c")                                                              #ctrl+с
        sleep(0.5)
        os.system("xdotool mousemove --sync %d %d click 1" % (clearGoogleInputFiledX, clearGoogleInputFiledY))                  #clear google input filed
        sleep(0.5)
        os.system("xdotool mousemove --sync %d %d click 1" % (startStopVoiceRecognitionX, startStopVoiceRecognitionY))          #start/stop voice recognition
        sleep(0.5)
        os.system("xdotool mousemove --sync 0 0")                                                                               #move mouse out

        pipe = Popen(["xclip", "-o"], stdout=PIPE)
        GoogleResult = pipe.communicate()[0]

        Popen(["xclip", "-i", "/dev/null"], stdout=PIPE)

        if GoogleResult:
            GoogleResult = GoogleResult.decode('utf-8').lower()
            GoogleResult = GoogleResult.encode('utf-8')
            print "GoogleResult:" + GoogleResult
            sayToEliza(GoogleResult, True)

if __name__ == "__main__":
    main()
