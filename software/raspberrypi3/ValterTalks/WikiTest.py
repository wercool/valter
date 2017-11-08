#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import wikipedia

wikipedia.set_lang("ru")

def main():
    print "Wiki Response Test"
    WikiResponse = wikipedia.summary("танк", sentences=5)
    print WikiResponse
    sys.exit(1)

if __name__ == "__main__":
    main()
