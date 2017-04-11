#-------------------------------------------------
#
# Project created by QtCreator 2017-04-11T15:04:31
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = contour
TEMPLATE = app

#pkg-config --cflags opencv
INCLUDEPATH += /usr/local/include/opencv

#pkg-config --libs opencv
LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc

SOURCES += main.cpp\
        mainwindow.cpp \
    cvimagewidget.cpp \
    imagemanipulator.cpp

HEADERS  += mainwindow.h \
    cvimagewidget.h \
    utils.h \
    imagemanipulator.h

FORMS    += mainwindow.ui

DISTFILES += \
    images/sample1.jpeg
