#-------------------------------------------------
#
# Project created by QtCreator 2016-02-29T19:45:39
#
#-------------------------------------------------

QT       += core
QT       += gui
QT       += quick
QT       += qml

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
           bodycontrolp1.cpp \
           armcontrolleft.cpp \
           armcontrolright.cpp \
           valter3d.cpp \
#service code
           platformcontrolp1.utils.cpp \
           bodycontrolp1.utils.cpp \
           armcontrolleft.utils.cpp \
           armcontrolright.utils.cpp \
           platformmanipulatorandirbumper.utils.cpp \
#mainwindow gui service code
           mainwindow/mainwindow.control.devices.utils.cpp \
           mainwindow/mainwindow.platformcontrolp1.cpp \
           mainwindow/mainwindow.platformlocationp1.cpp \
           mainwindow/mainwindow.platformcontrolp2.cpp \
           mainwindow/mainwindow.manipulatorandirbumper.cpp \
           mainwindow/mainwindow.bodycontrolp1.cpp \
           mainwindow/mainwindow.armcontrolleft.cpp \
           mainwindow/mainwindow.armcontrolright.cpp \
#tcp handlers
           tcphandlers/platformcontrolp1.tcphandler.cpp \
#tcp
            tcpinterface.cpp \
            tcp/threads/thread.cpp \
            tcp/tcpsockets/tcpstream.cpp \
            tcp/tcpsockets/tcpacceptor.cpp \
            tcp/tcpsockets/tcpconnector.cpp \
            tcphandlers/platformcontrolp2.tcphandler.cpp \
            tcphandlers/platformlocationp1.tcphandler.cpp \
            tcphandlers/platformmanipulatorandirbumper.tcphandler.cpp \
            tcphandlers/bodycontrolp1.tcphandler.cpp \
            tcphandlers/armcontrolright.tcphandler.cpp \
            tcphandlers/armcontrolleft.tcphandler.cpp

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
            armcontrolleft.h \
            armcontrolright.h \
            gui/guihelpers.h \
            gui/controldevicesGUI.h \
            gui/platformcontrolp1GUI.h \
            gui/platformcontrolp2GUI.h \
            gui/platformlocationp1GUI.h \
            gui/platformmanipulatorandirbumperGUI.h \
            gui/link1endpointviewitem.h \
            gui/link2endpointviewitem.h \
            gui/link3endpointviewitem.h \
            gui/bodycontrolp1GUI.h \
            gui/armcontrolleftGUI.h \
            gui/armcontrolrightGUI.h \
            valter3d.h \
#tcp
            tcpinterface.h \
            tcp/threads/thread.h \
            tcp/wqueue/wqueue.h \
            tcp/tcpsockets/tcpstream.h \
            tcp/tcpsockets/tcpacceptor.h \
            tcp/tcpsockets/tcpconnector.h \
            tcp/tcpsockets/workitem.h

FORMS    += mainwindow.ui

DISTFILES +=

CONFIG += c++11

RESOURCES += \
            resources/platform-location.qrc \
            valter3d.qrc

QMAKE_CXXFLAGS += -std=c++0x -D_GLIBCXX_USE_NANOSLEEP
