#-------------------------------------------------
#
# Project created by QtCreator 2016-02-29T19:45:39
#
#-------------------------------------------------

QT       += core
QT       += gui
#QT       += opengl

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
           platformcontrolp2.cpp \
           platformmanipulatorandirbumper.cpp \
           bodycontrolp1.cpp

HEADERS  += mainwindow.h \
            controldevice.h \
            valter.h \
            serial/include/serial/impl/unix.h \
            serial/include/serial/serial.h \
            serial/include/serial/v8stdint.h \
            platformcontrolp1.h \
            ivaltermodule.h \
            platformlocationp1.h \
            platformcontrolp2.h \
            platformmanipulatorandirbumper.h \
            bodycontrolp1.h \
            gui/guihelpers.h \
            gui/platformcontrolp1GUI.h \
            gui/platformcontrolp2GUI.h \
            gui/platformlocationp1GUI.h \
            gui/platformmanipulatorandirbumperGUI.h \
            gui/link1endpointviewitem.h \
            gui/link2endpointviewitem.h \
    gui/link3endpointviewitem.h

FORMS    += mainwindow.ui

DISTFILES +=

CONFIG += c++11

RESOURCES += \
    resources/platform-location.qrc
