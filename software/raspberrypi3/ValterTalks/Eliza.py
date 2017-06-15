# -*- coding: utf-8 -*-

import re
import random

reflections = {
    "am": "are",
    "was": "were",
    "i": "you",
    "i'd": "you would",
    "i've": "you have",
    "i'll": "you will",
    "my": "your",
    "are": "am",
    "you've": "I have",
    "you'll": "I will",
    "your": "my",
    "yours": "mine",
    "you": "me",
    "me": "you"
}

psychobabble = [
    [r'(.*)привет',
     ["привет. <break time=\"500ms\"/>я говорю на русском языке с ужасным акцентом.<break time=\"500ms\"/> извините. ",
      "привет"]],

    [r'(.*)как дела',
     ["у меня пока нет никаких дел",
      "всё как всегда",
      "настоящих дел нет - я только учусь",
      "я пока мало что умею"]],

    [r'(.*)ты умеешь',
     ["я только учусь",
      "я умею говорить, но плохо понимаю",
      "я умею распознавать некоторые предметы"]],

    [r'(.*)погворить(.*)',
     ["о чём вы хотите поговорить",
      "о чём мы поговорим",
      "я не очень хорошо уменю говорить на русском языке",
      "я не самы хороший собеседник"]],

    [r'(.*)кто(.*)(сделал|создал)',
     ["имя моего создателя Алексей",
      "у меня есть Создатель и его имя Алексей Майстренко",]],

    [r'(.*)как тебя зовут',
     ["{0} меня зовут Вальтер"]],

    [r'(.*)ты кто',
     ["я роб от"]],

    [r'(.*)что такое (.*)',
     ["wiki:{1}"]],

    [r'(.*)робот',
     ["даааааа, я роб от",
      "так точно - я робот"]],

    [r'(.*)(время|времени|который час)',
     ["time"]],

    [r'(.*)',
     ["может быть поговор им о чём-нибудь друг ом?",
      "извините, я вас не понял",
      "пожалуйста, продолжайте",
      "что-то я не очень хорош о вас понял"]],
]

def reflect(fragment):
    tokens = fragment.lower().split()
    for i, token in enumerate(tokens):
        if token in reflections:
            tokens[i] = reflections[token]
    return ' '.join(tokens)


def analyze(statement):
    for pattern, responses in psychobabble:
        match = re.match(pattern, statement.rstrip(".!"))
        if match:
            response = random.choice(responses)
            return response.format(*[reflect(g) for g in match.groups()])
