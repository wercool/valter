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
    inline static QPixmap cvMatToPixMap(cv::Mat mat)
    {
        QPixmap pixmap = QPixmap::fromImage(QImage(reinterpret_cast<uchar const*>(mat.data),
                                                  mat.cols,
                                                  mat.rows,
                                                  QImage::Format_Grayscale8));
        return pixmap;
    }

    void updateEnvPixmap();
    void activateLife(bool state);

    void on_loadEnvMapButton_clicked();

    void on_startLifeButton_clicked(bool checked);

    void on_populateColonyButton_clicked();

    void on_generateEnvMapButton_clicked();

    void on_killColonyButton_clicked();

    void on_updateEnvMapButton_clicked();

private:
    Ui::MainWindow *ui;

    QGraphicsScene *graphicsViewScene;

    cv::Mat envMapOriginalMat;
    int envMapOriginalMatBrightness = 0;
    cv::Mat envMapMat;

    QGraphicsPixmapItem *envMapPixmapItem;

    std::mutex envMapMutex;

    QTimer *lifeTimer;

    Colony colony;
};

#endif // MAINWINDOW_H
