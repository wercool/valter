#-------------------------------------------------
#
# Project created by QtCreator 2016-02-29T19:45:39
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET   = expclient
TEMPLATE = app


SOURCES += main.cpp\
           mainwindow.cpp \
           controldevice.cpp \
           valter.cpp \
           serial/src/impl/list_ports/list_ports_linux.cc \
           serial/src/impl/unix.cc \
           serial/src/serial.cc \
    platformcontrolp1.cpp \
    platformlocationp1.cpp \
    platformcontrolp2.cpp

HEADERS  += mainwindow.h \
            controldevice.h \
            valter.h \
            include/serial/impl/unix.h \
            include/serial/serial.h \
            include/serial/v8stdint.h \
            serial/include/serial/impl/unix.h \
            serial/include/serial/serial.h \
            serial/include/serial/v8stdint.h \
    platformcontrolp1.h \
    ivaltermodule.h \
    platformlocationp1.h \
    platformcontrolp2.h

FORMS    += mainwindow.ui

DISTFILES +=

CONFIG += c++11
