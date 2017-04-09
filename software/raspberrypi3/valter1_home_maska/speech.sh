#!/bin/bash

if [[ $1 =~ .*"как тебя зовут".* ]]; then
    text_to_speech="Меня зовут Вальтер. Я робот. А как тебя зовут?"
elif [[ $1 =~ .*"как твое имя".* ]]; then
    text_to_speech="Моё имя Вальтер. Я антропоморфный робот. А как тебя зовут?"
elif [[ $1 =~ .*"кто теб"[я,е].*(создал)|(сделал).* ]]; then
    text_to_speech="Моего создателя зовут Алексей Майстренко."
elif [[ $1 =~ .*(меня зовут).* ]]; then
    parts=( $1 )
    text_to_speech="Привет, ${parts[2]}. Приятно познакомиться."
else
    text_to_speech=$1
fi

echo $text_to_speech
encoded_text=`python -c "import sys, urllib as ul; print ul.quote_plus('$text_to_speech')"`
echo $encoded_text

wget "https://tts.voicetech.yandex.net/generate?text=$text_to_speech&format=wav&lang=ru-RU&speaker=ermil&emotion=good&key=069b6659-984b-4c5f-880e-aaedcfd84102" -O asr-result.wav
aplay asr-result.wav -D sysdefault:CARD=4 -c2



#cvlc -q --play-and-exit "https://tts.voicetech.yandex.net/generate?text=$encoded_text&format=mp3&lang=ru-RU&speaker=ermil&emotion=good&key=069b6659-984b-4c5f-880e-aaedcfd84102"

