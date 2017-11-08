#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import wikipedia
import re

wikipedia.set_lang("ru")

def main():
    print >> sys.stderr, "possible error test"

    print "Wiki Response Test: танк"
    WikiResponse = wikipedia.summary("танк", sentences=5)
    WikiResponse = re.sub(ur'.*?\((.*?)\)', ' ', WikiResponse)
    WikiResponse = re.sub(ur'\(.*?\) —', '', WikiResponse)
    first_dot_index = WikiResponse.find('.')
    WikiResponse = WikiResponse[0: first_dot_index]
    print WikiResponse

    print "Wiki Response Test: транзистор"
    WikiResponse = wikipedia.summary("транзистор", sentences=5)
    WikiResponse = re.sub(ur'\((.*?)\)', ' ', WikiResponse)
    WikiResponse = re.sub(ur'\(.*?\) —', ' ', WikiResponse)
    first_dot_index = WikiResponse.find('.')
    WikiResponse = WikiResponse[0: first_dot_index]
    print WikiResponse

    sys.exit(1)

if __name__ == "__main__":
    main()
