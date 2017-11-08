#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
from subprocess import Popen, PIPE
from time import sleep
from time import localtime, strftime
import os
import re
import sys
import wikipedia
import Eliza

wikipedia.set_lang("ru")

def sayToEliza(statement, inVoice):
    global stop
    ElizaAnswer = Eliza.analyze(statement)
    if ElizaAnswer:
        if ElizaAnswer == "stop_recognition":
            stop = True
            pipe = Popen(["bash", "/home/maska/speech-ru", '"распознавание завершено"'], stdout=PIPE)
            pipe.communicate()
        if ElizaAnswer == "time":
            hours = strftime("%H", localtime())
            minutes = strftime("%M", localtime())
            ElizaAnswer = "Сейчас " + hours + " часов " + minutes + " минут"
        if ElizaAnswer.find("wiki:") > -1:
            try:
                WikiResponse = wikipedia.summary(ElizaAnswer[4:], sentences=5)
                print "Wiki Response: " + WikiResponse
                try:
                    if WikiResponse:
                        ElizaAnswer = re.sub(ur'\(.*?\) —', '', WikiResponse)
                        first_dot_index = ElizaAnswer.find('.')
                        ElizaAnswer = ElizaAnswer[0: first_dot_index]
                except:
                    ElizaAnswer = "извините, я не могу ответить на ваш вопрос. продолжайте, пожалуйста."
            except:
                ElizaAnswer = "извините, я затрудняюсь однозначно ответить на ваш вопрос. возможно вам стоит уточнить его."

        print "> " + ElizaAnswer

        if inVoice and not(stop):
            if ElizaAnswer:
                pipe = Popen(["bash", "/home/maska/speech-ru", '"' + ElizaAnswer + '"'], stdout=PIPE)
                pipe.communicate()

stop = False
browser = 0

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

    #Valter
    startStopVoiceRecognitionX = 47
    startStopVoiceRecognitionY = 387
    clearGoogleInputFiledX = 492
    clearGoogleInputFiledY = 290
    centerGoogleInputFiledX = 255
    centerGoogleInputFiledY = 335
    #laboratory
    """
    startStopVoiceRecognitionX = 2020
    startStopVoiceRecognitionY = 411
    clearGoogleInputFiledX = 2621
    clearGoogleInputFiledY = 317
    centerGoogleInputFiledX = ?
    centerGoogleInputFiledY = ?
    """
    #office
    """
    startStopVoiceRecognitionX = 1373
    startStopVoiceRecognitionY = 376
    clearGoogleInputFiledX = 1973
    clearGoogleInputFiledY = 279
    centerGoogleInputFiledX = ?
    centerGoogleInputFiledY = ?
    """

    pipe = Popen(["bash", "/home/maska/speech-ru", '"Подгот овка системы распознав ания русской речи"'], stdout=PIPE)
    global browser 
    browser = Popen(["chromium-browser", "--start-maximized", "https://translate.google.com/#ru/en"], stdout=PIPE)
    sleep(10.0)
    pipe = Popen(["bash", "/home/maska/speech-ru", '"Я гот ов"'], stdout=PIPE)

    global stop

    while not(stop):
        GoogleResult = ""

        pipe = Popen(["aplay", "/home/maska/git/valter/software/raspberrypi3/ValterTalks/sounds/beep1.wav", "-D", "sysdefault:CARD=Set"], stdout=PIPE)
        os.system("xdotool mousemove --sync %d %d click 1" % (startStopVoiceRecognitionX, startStopVoiceRecognitionY))          #start/stop voice recognition

        sleep(5.0)

        os.system("xdotool mousemove --sync %d %d click 1" % (centerGoogleInputFiledX, centerGoogleInputFiledY))                #center google input filed
        sleep(0.5)
        os.system("xdotool key --clearmodifiers ctrl+a")                                                              #ctrl+a
        sleep(0.5)
        os.system("xdotool key --clearmodifiers ctrl+c")                                                              #ctrl+с
        sleep(0.5)
        os.system("xdotool mousemove --sync %d %d click 1" % (clearGoogleInputFiledX, clearGoogleInputFiledY))                  #clear google input filed
        sleep(0.5)
        os.system("xdotool mousemove --sync %d %d click 1" % (startStopVoiceRecognitionX, startStopVoiceRecognitionY))          #start/stop voice recognition
        sleep(0.5)
        os.system("xdotool mousemove --sync %d %d click 1" % (clearGoogleInputFiledX, clearGoogleInputFiledY))                  #clear google input filed
        sleep(0.5)

        pipe = Popen(["xclip", "-o"], stdout=PIPE)
        GoogleResult = pipe.communicate()[0]

        Popen(["xclip", "-i", "/dev/null"], stdout=PIPE)

        if GoogleResult:
            GoogleResult = GoogleResult.decode('utf-8').lower()
            GoogleResult = GoogleResult.encode('utf-8')
            print "GoogleResult:" + GoogleResult
            sayToEliza(GoogleResult, True)

    browser.kill()
    sys.exit(1)

if __name__ == "__main__":
    main()
