#-------------------------------------------------
#
# Project created by QtCreator 2017-04-11T15:04:31
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = imagelab
TEMPLATE = app

#pkg-config --cflags opencv
INCLUDEPATH += /usr/local/include/opencv

#pkg-config --libs opencv
LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs\
                         -lopencv_highgui -lopencv_imgproc\
                         -lopencv_video -lopencv_videoio\
                         -lopencv_objdetect -lopencv_features2d\
                         -lopencv_xfeatures2d -lopencv_calib3d

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
    images/sample1.jpeg \
    images/videocam.png


QMAKE_CXXFLAGS += -std=c++0x -D_GLIBCXX_USE_NANOSLEEP

RESOURCES += \
    images.qrc
