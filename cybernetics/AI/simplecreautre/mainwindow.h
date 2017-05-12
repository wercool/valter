#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QToolButton>
#include <QTimer>

#include "colony.h"
#include "dline.h"

QT_BEGIN_NAMESPACE
class QGraphicsScene;
QT_END_NAMESPACE

namespace Ui {
class MainWindow;
}

class MainWindow : public QWidget
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);

    void populateColony();

private:
    QGraphicsScene *scene;
    QTimer *lifeTimer;
    QToolButton *startButton;

    Colony *colony;

    // debug geometry
    DLine *dline1;
    DLine *dline2;

    DLine *dlineX;
    DLine *dlineY;

private slots:
    void startLifeCallback(bool state);
    void lifeTimerCallback();
    void addDebugGeometry();

};

#endif // MAINWINDOW_H
