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

def main():)
    question = sys.argv[1]
    print("Question: " + question)
    
    answer = question
    
    pipe = Popen(["bash", "/home/maska/say.sh", '"' + answer + '"'], stdout=PIPE)
    pipe.communicate()

if __name__ == "__main__":
    main()
