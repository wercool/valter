#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import wikipedia
import re
import subprocess
from subprocess import Popen, PIPE

wikipedia.set_lang("ru")

reload(sys)
sys.setdefaultencoding('utf-8')

def main():
    question = sys.argv[1]
    print("Question: " + question)

    try:
        WikiResponse = wikipedia.summary(question, sentences=5)
        print("Wiki responds: " + WikiResponse)

        Answer = re.sub(ur'.*?\((.*?)\)', ' ', WikiResponse)
        Answer = re.sub(ur'\(.*?\) —', '', WikiResponse)
        #first_dot_index = Answer.find('.')
        #Answer = WikiResponse[0: first_dot_index]
        Answer = re.sub(ur'—', '', Answer)
        Answer = re.sub(ur'=', '', Answer)
        Answer = re.sub(ur'«', '', Answer)
        Answer = re.sub(ur'»', '', Answer)
        Answer = re.sub(ur',', '', Answer)
        Answer = re.sub(ur'а́', 'а', Answer)
        Answer = re.sub(ur'о́', 'о', Answer)
        Answer = re.sub(ur'е́', 'е', Answer)
        Answer = re.sub(ur'и́', 'и', Answer)
        Answer = re.sub(ur'я́', 'я', Answer)
        Answer = re.sub(ur'ю́', 'ю', Answer)
        Answer = re.sub(ur'у́', 'у', Answer)

        Answer = re.sub(ur'ў', 'в', Answer)
        Answer = re.sub(ur'і', 'и', Answer)

        Answer = Answer.strip()
        print >> sys.stdout, "Answer: \n" + Answer
    except Exception, e:
        Answer = "извините я не могу ответить на ваш вопрос продолжайте пожалуйста"

    pipe = Popen(["bash", "/home/maska/say.sh", Answer], stdout=PIPE)
    pipe.communicate()

if __name__ == "__main__":
    main()
