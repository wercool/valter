#!/bin/bash

VALTER_TALKS=$(ps -ef | grep 'ValterTalks.py' | grep -v "grep" | wc -l)

if [ "$VALTER_TALKS" -eq "1" ]; then
    echo "Valter Talks already running..."
else
    /home/maska/git/valter/software/raspberrypi3/ValterTalks/ValterTalks.py &
fi
