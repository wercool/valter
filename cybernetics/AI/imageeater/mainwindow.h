#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QGraphicsPixmapItem>
#include <QTimer>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <utils/perlinnoisemat.h>
#include <creature/colony.h>

#include <thread>
#include <mutex>

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
    void lifeTimerCallback();
    void generatePerlinNoiseEnvMap(cv::Size size, double scale);
    QPixmap cvMatToPixMap(cv::Mat mat);

    void updateEnvPixmap();
    void activateLife(bool state);

    void on_loadEnvMapButton_clicked();

    void on_startLifeButton_clicked(bool checked);

    void on_populateColonyButton_clicked();

    void on_generateEnvMapButton_clicked();

    void on_killColonyButton_clicked();

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
