#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QGraphicsPixmapItem>
#include <QTimer>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <creature/colony.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_loadEnvMapButton_clicked();
    void lifeTimerCallback();

    void on_startLifeButton_clicked(bool checked);

    void on_populateColonyButton_clicked();

private:
    Ui::MainWindow *ui;

    QGraphicsScene *graphicsViewScene;

    cv::Mat envMapOriginalMat;
    cv::Mat envMapMat;

    QGraphicsPixmapItem *envMapPixmapItem;

    std::mutex envMapMutex;

    QTimer *lifeTimer;

    Colony colony;
};

#endif // MAINWINDOW_H
