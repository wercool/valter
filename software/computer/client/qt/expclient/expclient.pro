#-------------------------------------------------
#
# Project created by QtCreator 2016-02-29T19:45:39
#
#-------------------------------------------------

QT       += core
QT       += gui
QT       += quick
QT       += websockets

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
#service code
           mainclassesutils/platformcontrolp1.utils.cpp \
           mainclassesutils/bodycontrolp1.utils.cpp \
           mainclassesutils/armcontrolleft.utils.cpp \
           mainclassesutils/armcontrolright.utils.cpp \
           mainclassesutils/platformmanipulatorandirbumper.utils.cpp \
#mainwindow gui service code
           mainwindow/mainwindow.control.devices.utils.cpp \
           mainwindow/mainwindow.platformcontrolp1.cpp \
           mainwindow/mainwindow.platformlocationp1.cpp \
           mainwindow/mainwindow.platformcontrolp2.cpp \
           mainwindow/mainwindow.manipulatorandirbumper.cpp \
           mainwindow/mainwindow.bodycontrolp1.cpp \
           mainwindow/mainwindow.armcontrolleft.cpp \
           mainwindow/mainwindow.armcontrolright.cpp \
#tcp
            tcpinterface.cpp \
            tcp/threads/thread.cpp \
            tcp/tcpsockets/tcpstream.cpp \
            tcp/tcpsockets/tcpacceptor.cpp \
            tcp/tcpsockets/tcpconnector.cpp \
#tcp handlers
            tcphandlers/platformcontrolp1.tcphandler.cpp \
            tcphandlers/platformcontrolp2.tcphandler.cpp \
            tcphandlers/platformlocationp1.tcphandler.cpp \
            tcphandlers/platformmanipulatorandirbumper.tcphandler.cpp \
            tcphandlers/bodycontrolp1.tcphandler.cpp \
            tcphandlers/armcontrolright.tcphandler.cpp \
            tcphandlers/armcontrolleft.tcphandler.cpp \
            tcphandlers/taskmanager.tcphandler.cpp \
#tasks
            tasks/itask.cpp \
            tasks/taskmanager.cpp \
            tasks/generic/delaytask.cpp \
            tasks/generic/setmoduleinitialstatetask.cpp \
            tasks/platform-manipulator-and-ir-bumper/setlink1positiontask.cpp \
            tasks/platform-manipulator-and-ir-bumper/setlink2positiontask.cpp \
            tasks/platform-manipulator-and-ir-bumper/setgrippertiltpositiontask.cpp \
            tasks/platform-manipulator-and-ir-bumper/setgripperrotationposition.cpp \
            tasks/platform-manipulator-and-ir-bumper/setgrippergrasperposition.cpp \
            tasks/platform-manipulator-and-ir-bumper/setlink1motordynamics.cpp \
            tasks/platform-manipulator-and-ir-bumper/setlink2motordynamics.cpp \
            tasks/platform-manipulator-and-ir-bumper/setgripperrotationmotordynamics.cpp \
            tasks/platform-control-p1/trasnslateplatformlinearlytask.cpp \
            tasks/platform-control-p1/rotateplatformtask.cpp \
            tasks/platform-control-p1/rotatebodytask.cpp \
            tasks/arm-control-right/setrightforearmpositiontask.cpp \
            tasks/arm-control-right/setrightarmpositiontask.cpp \
            tasks/arm-control-right/setrightlimbpositiontask.cpp \
            tasks/arm-control-right/setrightarmrollpositiontask.cpp \
            tasks/arm-control-right/righthandgriptask.cpp \
            tasks/arm-control-left/setleftforearmpositiontask.cpp \
            tasks/arm-control-left/setleftarmpositiontask.cpp \
            tasks/arm-control-left/setleftlimbpositiontask.cpp \
            tasks/arm-control-left/lefthandgriptask.cpp \
            tasks/body-control-p1/setrightarmyawpositiontask.cpp \
            tasks/body-control-p1/setleftarmyawpositiontask.cpp \
            tasks/platform-control-p1/cmdveltask.cpp \
            tasks/platform-control-p1/translateplatformtwistytask.cpp \
            tasks/arm-control-left/setleftarmrollpositiontask.cpp \
            tasks/body-control-p1/setheadyawpositiontask.cpp \
            tasks/body-control-p1/setheadpitchpositiontask.cpp \
            tasks/generic/saytask.cpp \
            tasks/generic/voicerecognitiontask.cpp\
            serial/src/impl/list_ports/list_ports_osx.cc \
            serial/src/impl/list_ports/list_ports_win.cc \
            serial/src/impl/win.cc \
#websockets
            websocketserver.cpp

HEADERS  += mainwindow.h \
            controldevice.h \
            valter.h \
            ivaltermodule.h \
            serial/include/serial/impl/unix.h \
            serial/include/serial/serial.h \
            serial/include/serial/v8stdint.h \
            platformcontrolp1.h \
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
            gui/taskstabgui.h \
#tcp
            tcpinterface.h \
            tcp/threads/thread.h \
            tcp/wqueue/wqueue.h \
            tcp/tcpsockets/tcpstream.h \
            tcp/tcpsockets/tcpacceptor.h \
            tcp/tcpsockets/workitem.h \
            tcp/tcpsockets/tcpconnector.h \
#tasks
            tasks/itask.h \
            tasks/tasks.h \
            tasks/taskmanager.h \
            tasks/generic/delaytask.h \
            tasks/generic/setmoduleinitialstatetask.h \
            tasks/platform-manipulator-and-ir-bumper/setlink2positiontask.h \
            tasks/platform-manipulator-and-ir-bumper/setlink1positiontask.h \
            tasks/platform-manipulator-and-ir-bumper/setgrippertiltpositiontask.h \
            tasks/platform-manipulator-and-ir-bumper/setgripperrotationposition.h \
            tasks/platform-manipulator-and-ir-bumper/setgrippergrasperposition.h \
            tasks/platform-manipulator-and-ir-bumper/setlink1motordynamics.h \
            tasks/platform-manipulator-and-ir-bumper/setlink2motordynamics.h \
            tasks/platform-manipulator-and-ir-bumper/setgripperrotationmotordynamics.h \
            tasks/platform-control-p1/trasnslateplatformlinearlytask.h \
            tasks/platform-control-p1/rotateplatformtask.h \
            tasks/platform-control-p1/rotatebodytask.h \
            tasks/arm-control-right/setrightforearmpositiontask.h \
            tasks/arm-control-right/setrightarmpositiontask.h \
            tasks/arm-control-right/setrightlimbpositiontask.h \
            tasks/arm-control-right/setrightarmrollpositiontask.h \
            tasks/arm-control-right/righthandgriptask.h \
            tasks/arm-control-left/setleftforearmpositiontask.h \
            tasks/arm-control-left/setleftarmpositiontask.h \
            tasks/arm-control-left/setleftlimbpositiontask.h \
            tasks/arm-control-left/lefthandgriptask.h \
            tasks/body-control-p1/setrightarmyawpositiontask.h \
            tasks/body-control-p1/setleftarmyawpositiontask.h \
            tasks/platform-control-p1/cmdveltask.h \
            tasks/arm-control-left/setleftarmrollpositiontask.h \
            tasks/body-control-p1/setheadyawpositiontask.h \
            tasks/body-control-p1/setheadpitchpositiontask.h \
            tasks/generic/saytask.h \
            tasks/generic/voicerecognitiontask.h \
            tasks/platform-control-p1/translateplatformtwistytask.h \
            serial/include/serial/impl/win.h \
#websockets
            websocketserver.h

FORMS    += mainwindow.ui

DISTFILES += \
    resources/3rdparty/gl-matrix.js \
    resources/3rdparty/ThreeJSLoader.js \
    resources/valter_model_json/textures/dark_aluminum.jpg \
    valter_head_icon.png \
    resources/commands/ARM-CONTROL-LEFT \
    resources/commands/ARM-CONTROL-RIGHT \
    resources/commands/BODY-CONTROL-P1 \
    resources/commands/PLATFORM-CONTROL-P1 \
    resources/commands/PLATFORM-CONTROL-P2 \
    resources/commands/PLATFORM-LOCATION-P1 \
    resources/commands/PLATFORM-MANIPULATOR-AND-IR-BUMPER \
    resources/settings/arm-control-left-defaults \
    resources/settings/arm-control-right-defaults \
    resources/settings/body-control-p1-defaults \
    resources/settings/global-settings \
    resources/settings/platform-control-p1-defaults \
    resources/settings/platform-control-p2-defaults \
    resources/settings/platform-location-p1-defaults \
    resources/settings/platform-manipulator-and-ir-bumper-defaults \
    resources/tasks/ARM-CONTROL-LEFT \
    resources/tasks/ARM-CONTROL-RIGHT \
    resources/tasks/BODY-CONTROL-P1 \
    resources/tasks/GENERIC \
    resources/tasks/PLATFORM-CONTROL-P1 \
    resources/tasks/PLATFORM-MANIPULATOR-AND-IR-BUMPER \
    resources/tasks/predefined_tasks

CONFIG += c++11

RESOURCES += \
            resources/platform-location.qrc \

QMAKE_CXXFLAGS += -std=c++0x -D_GLIBCXX_USE_NANOSLEEP
