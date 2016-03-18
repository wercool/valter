#ifndef PLATFORMLOCATIONP1GUI_H
#define PLATFORMLOCATIONP1GUI_H

#include <QMainWindow>
#include <QTimer>
#include <QtDebug>
#include <QMessageBox>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <valter.h>

void initLedButtons(MainWindow *mainWindow)
{
    mainWindow->getUi()->ch0RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch1RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch2RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch3RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch4RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch5RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch6RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch7RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch8RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch9RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch10RedLed->setIcon(mainWindow->redLedOffIcon);
    mainWindow->getUi()->ch11RedLed->setIcon(mainWindow->redLedOffIcon);

    mainWindow->getUi()->ch0GreenLed->setIcon(mainWindow->greenLefOffIcon);
    mainWindow->getUi()->ch1GreenLed->setIcon(mainWindow->greenLefOffIcon);
    mainWindow->getUi()->ch2GreenLed->setIcon(mainWindow->greenLefOffIcon);
    mainWindow->getUi()->ch3GreenLed->setIcon(mainWindow->greenLefOffIcon);
    mainWindow->getUi()->ch4GreenLed->setIcon(mainWindow->greenLefOffIcon);
    mainWindow->getUi()->ch5GreenLed->setIcon(mainWindow->greenLefOffIcon);
    mainWindow->getUi()->ch6GreenLed->setIcon(mainWindow->greenLefOffIcon);
    mainWindow->getUi()->ch7GreenLed->setIcon(mainWindow->greenLefOffIcon);
    mainWindow->getUi()->ch8GreenLed->setIcon(mainWindow->greenLefOffIcon);
    mainWindow->getUi()->ch9GreenLed->setIcon(mainWindow->greenLefOffIcon);
    mainWindow->getUi()->ch10GreenLed->setIcon(mainWindow->greenLefOffIcon);
    mainWindow->getUi()->ch11GreenLed->setIcon(mainWindow->greenLefOffIcon);
}

void setRedLedButtonOn(MainWindow *mainWindow, QPushButton *btn, bool state)
{
    if (state)
    {
        btn->setIcon(mainWindow->redLedOnIcon);
    }
    else
    {
        btn->setIcon(mainWindow->redLedOffIcon);
    }
}

void setRedLedButtonOff(MainWindow *mainWindow, QPushButton *btn)
{
    btn->setIcon(mainWindow->redLedOffIcon);
}

void setGreenLedButtonOn(MainWindow *mainWindow, QPushButton *btn, bool state)
{
    if (state)
    {
        btn->setIcon(mainWindow->greenLefOnIcon);
    }
    else
    {
        btn->setIcon(mainWindow->greenLefOffIcon);
    }
}

void setGreenLedButtonOff(MainWindow *mainWindow, QPushButton *btn)
{
    btn->setIcon(mainWindow->greenLefOffIcon);
}

#endif // PLATFORMLOCATIONP1GUI_H
