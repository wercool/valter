#ifndef CVIMAGEWIDGET_H
#define CVIMAGEWIDGET_H

#include <QWidget>
#include <QImage>
#include <QPainter>
#include <opencv2/opencv.hpp>

class CVImageWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CVImageWidget(QWidget *parent = 0);

    QSize sizeHint() const { return _qimage.size(); }
    QSize minimumSizeHint() const { return _qimage.size(); }

signals:

public slots:
    void showImage(const cv::Mat& image);

protected:
    void paintEvent(QPaintEvent* /*event*/);

    QImage _qimage;
    cv::Mat _tmp;
};

#endif // CVIMAGEWIDGET_H
