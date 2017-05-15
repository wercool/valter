#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QToolButton>
#include <QPushButton>
#include <QFileDialog>
#include <QTimer>

#include "view.h"

#include "perlinnoiseenvironment.h"
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
    void killColony();

private:
    View *view;
    QGraphicsScene *scene;
    QTimer *lifeTimer;

    QPushButton *perlinNoiseEnvironmentMapGenerateButton;
    QPushButton *loadImageButton;
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
    void generateEnvironmentMap();
    void loadImageCallback();
    void addDebugGeometry();

};

#endif // MAINWINDOW_H
