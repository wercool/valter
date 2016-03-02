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
           serial/src/impl/list_ports/list_ports_osx.cc \
           serial/src/impl/list_ports/list_ports_win.cc \
           serial/src/impl/unix.cc \
           serial/src/impl/win.cc \
           serial/src/serial.cc

HEADERS  += mainwindow.h \
            controldevice.h \
            valter.h \
            include/serial/impl/unix.h \
            include/serial/impl/win.h \
            include/serial/serial.h \
            include/serial/v8stdint.h \
            serial/include/serial/impl/unix.h \
            serial/include/serial/impl/win.h \
            serial/include/serial/serial.h \
            serial/include/serial/v8stdint.h

FORMS    += mainwindow.ui

DISTFILES += \
    valter_head_icon.png \
    resources/commands/PLATFORM_CONROL_P1 \
    resources/commands/BODY-CONTROL-P1

CONFIG += c++11
