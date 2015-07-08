#!/bin/bash
./UscCmd --servo 1,2820
./UscCmd --servo 2,3200
./UscCmd --servo 3,8000
./UscCmd --servo 4,8000
sleep 0.25
./UscCmd --servo 0,2440
./UscCmd --servo 5,9200

sleep 2

./UscCmd --servo 0,5200
./UscCmd --servo 5,6000
sleep 0.25
./UscCmd --servo 1,7400
./UscCmd --servo 2,8000
./UscCmd --servo 3,3600
./UscCmd --servo 4,3360



