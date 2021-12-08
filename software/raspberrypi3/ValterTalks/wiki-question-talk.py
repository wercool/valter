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
    
    WikiResponse = wikipedia.summary(question, sentences=5)
    WikiResponse = re.sub(ur'.*?\((.*?)\)', ' ', WikiResponse)
    WikiResponse = re.sub(ur'\(.*?\) —', '', WikiResponse)
    first_dot_index = WikiResponse.find('.')
    WikiResponse = WikiResponse[0: first_dot_index]
    print >> sys.stdout, "Wiki Response: \n" + WikiResponse
    
    answer = question
    
    pipe = Popen(["bash", "/home/maska/say.sh", '"' + WikiResponse + '"'], stdout=PIPE)
    pipe.communicate()

if __name__ == "__main__":
    main()
