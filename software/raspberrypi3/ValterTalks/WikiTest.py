#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import wikipedia
import re

wikipedia.set_lang("ru")

def main():

    print "Wiki Request: танк\n"
    WikiResponse = wikipedia.summary("танк", sentences=5)
    WikiResponse = re.sub(ur'.*?\((.*?)\)', ' ', WikiResponse)
    WikiResponse = re.sub(ur'\(.*?\) —', '', WikiResponse)
    first_dot_index = WikiResponse.find('.')
    WikiResponse = WikiResponse[0: first_dot_index]
    print >> sys.stdout, "Wiki Response: \n" + WikiResponse

    print "Wiki Request: транзистор\n"
    WikiResponse = wikipedia.summary("транзистор", sentences=5)
    WikiResponse = re.sub(ur'\((.*?)\)', ' ', WikiResponse)
    WikiResponse = re.sub(ur'\(.*?\) —', ' ', WikiResponse)
    first_dot_index = WikiResponse.find('.')
    WikiResponse = WikiResponse[0: first_dot_index]
    print >> sys.stdout, "Wiki Response: \n" + WikiResponse

    sys.exit(1)

if __name__ == "__main__":
    main()
