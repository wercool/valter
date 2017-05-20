#-------------------------------------------------
#
# Project created by QtCreator 2017-05-17T00:45:11
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

qtHaveModule(opengl): QT += opengl

QT += charts

TARGET = imageeater
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

#pkg-config --cflags opencv
INCLUDEPATH += /usr/local/include/opencv

#pkg-config --libs opencv
LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs\
                         -lopencv_highgui -lopencv_imgproc


SOURCES += main.cpp\
        mainwindow.cpp \
    graphicsview.cpp \
    creature/colony.cpp \
    creature/creaturegi.cpp \
    creature/creature.cpp \
    utils/perlinnoisemat.cpp \
    creature/neuron.cpp \
    creature/receptor.cpp \
    creature/creaturea.cpp \
    creature/neuralnetwork.cpp \
    creature/creatureb.cpp

HEADERS  += mainwindow.h \
    graphicsview.h \
    creature/colony.h \
    creature/creaturegi.h \
    creature/creature.h \
    utils/perlinnoisemat.h \
    creature/neuron.h \
    creature/receptor.h \
    creature/creaturea.h \
    creature/neuralnetwork.h \
    utils/generic-utils.h \
    creature/creatureb.h

FORMS    += mainwindow.ui

QMAKE_CXXFLAGS += -std=c++11
