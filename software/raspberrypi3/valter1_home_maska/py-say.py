#!/usr/bin/env python

from urllib2 import urlopen
#to python3.x
#from urllib.request import urlopen
import pyaudio


pyaud = pyaudio.PyAudio()

srate=44100

p = pyaudio.PyAudio()
for i in range(p.get_device_count()):
  dev = p.get_device_info_by_index(i)
  print((i,dev['name'],dev['maxInputChannels']))

stream = pyaud.open(format = pyaud.get_format_from_width(1),
                channels = 2,
                rate = srate,
                output = True,
                output_device_index=3)


url = "http://download.wavetlan.com/SVV/Media/HTTP/WAV/NeroSoundTrax/NeroSoundTrax_test4_PCM_Mono_VBR_8SS_44100Hz.wav"
u = urlopen(url)

data = u.read(8192)

while data:

    stream.write(data)
    data = u.read(8192)
