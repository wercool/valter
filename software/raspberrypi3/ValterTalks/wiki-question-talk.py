#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import wikipedia
import re

wikipedia.set_lang("ru")

reload(sys)
sys.setdefaultencoding('utf-8')

def main():
    print(sys.argv[1])

if __name__ == "__main__":
    main()
