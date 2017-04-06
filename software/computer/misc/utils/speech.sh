#!/bin/bash

encoded_text=`python -c "import sys, urllib as ul; print ul.quote_plus('$1')"`
echo $encoded_text

cvlc -q --play-and-exit "https://tts.voicetech.yandex.net/generate?text=$encoded_text&format=mp3&lang=ru-RU&speaker=ermil&emotion=good&key=069b6659-984b-4c5f-880e-aaedcfd84102"
