# -*- coding: utf-8 -*-

import re
import random

reflections = {
    "am": "are",
    "was": "were",
    "я": "вы",
    "вы": "я",
    "я бы": "вы бы",
    "i've": "you have",
    "i'll": "you will",
    "my": "your",
    "are": "am",
    "you've": "I have",
    "you'll": "I will",
    "your": "my",
    "yours": "mine"
}

psychobabble = [
    [r'(.*)завершить распознавание',
     ["stop_recognition"]],

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

    [r'(.*)поговорить(.*)',
     ["о чём вы хотите поговорить",
      "о чём мы поговорим",
      "я не очень хорошо уменю говорить на русском языке",
      "я не самы хороший собеседник"]],

    [r'(.*)кто(.*)(сделал|создал)',
     ["имя моего создателя Алексей",
      "у меня есть Создатель и его имя Алексей Майстренко",]],

    [r'(.*)как тебя зовут',
     ["{0} меня зовут Вальтер"]],

    [r'(.*)о себе(.*)',
     ["Я роб от. Имя моего создателя Алексей Майстренко. Он должен мной гордиться. Я только учусь понимать русский язык. Я плохо говорю на русском языке, извините. Но я много знаю."]],

    [r'(.*)ты кто',
     ["я роб от"]],

    [r'(.*)кто такой (Путин|путин)(.*)',
     ["Влади́мир Влади́мирович Пу́тин — советский и российский государственный и политический деятель, действующий президент Российской Федерации с 7-го мая 2012-го года, бывший исполняющий обязанности министра иностранных дел Украины Андрій Богданович Дещиця утверждает, что Владимир Владимирович Путин широко известен также как хуйл о."]],

    [r'(.*)что ты знаешь',
     ["Я плохо говорю на русском языке, извините. Но я много знаю."]],

    [r'(.*)знаешь про(.*)',
     ["wiki:{1}"]],

    [r'что такое метро',
     ["метро - это свинь<break time=\"20ms\"/>ти<break time=\"20ms\"/>гля"]],

    [r'как говорит мишутка',
     ["гугль гугль гугль гугль"]],

    [r'(.*)что такое(.*)',
     ["wiki:{1}"]],

    [r'(.*)кто такой(.*)',
     ["wiki:{1}"]],

    [r'(.*)что значит(.*)',
     ["wiki:{1}"]],

    [r'(.*)робот',
     ["даааааа, я роб от",
      "так точно - я робот"]],

    [r'(.*)(время|времени|который час)',
     ["time"]],

    [r'(.*)',
     ["может быть поговор им о чём-нибудь друг ом?",
      "извините, я вас не понял",
      "я не очень хорошо понимаю русскую речь, извините",
      "продолжайте, пожалуйста",
      "чтобы вам хотелось узнать",
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
