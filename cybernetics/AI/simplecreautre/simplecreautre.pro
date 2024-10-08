#-------------------------------------------------
#
# Project created by QtCreator 2017-05-10T02:04:04
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

qtHaveModule(opengl): QT += opengl

TARGET = simplecreautre
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
                         -lopencv_highgui -lopencv_imgproc\
                         -lopencv_video -lopencv_videoio\
                         -lopencv_objdetect -lopencv_features2d\
                         -lopencv_xfeatures2d -lopencv_calib3d\
                         -lopencv_photo

SOURCES += \
    main.cpp\
    mainwindow.cpp \
    view.cpp \
    creature.cpp \
    neuralnetwork.cpp \
    neuron.cpp \
    creaturea.cpp \
    creatureb.cpp \
    colony.cpp \
    receptor.cpp \
    dline.cpp \
    perlinnoiseenvironment.cpp

HEADERS  += \
    mainwindow.h \
    view.h \
    creature.h \
    neuralnetwork.h \
    neuron.h \
    creaturea.h \
    creatureb.h \
    colony.h \
    receptor.h \
    dline.h \
    perlinnoiseenvironment.h \
    utils/opencv-qt/opencvMatToQPixmap.h \
    utils/utils.h

FORMS    +=

QMAKE_CXXFLAGS += -D_GLIBCXX_USE_NANOSLEEP
QMAKE_CXXFLAGS += -std=c++11

DISTFILES += \
    geometry-help.txt

RESOURCES += \
    images.qrc
