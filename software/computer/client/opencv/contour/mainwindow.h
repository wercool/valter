#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "cvimagewidget.h"
#include "imagemanipulator.h"
#include "utils.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    ImageManipulator *imageManipulator;

    CVImageWidget* srcImageWidget;
    CVImageWidget* procImageWidget;

private slots:
    void on_brightnessHorizontalSlider_valueChanged(int value);

    void on_contrastHorizontalSlider_valueChanged(int value);

    void on_homogeneousBlurHorizontalSlider_valueChanged(int value);

    void on_gaussianBlurHorizontalSlider_valueChanged(int value);

    void on_medianBlurHorizontalSlider_valueChanged(int value);

    void on_bilateralBlurHorizontalSlider_valueChanged(int value);

    void on_processButton_clicked();

    void on_grayscaleCheckBox_clicked(bool checked);

    void on_findContoursButton_clicked();

    void on_cannyThresholdSlider_valueChanged(int value);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
