#ifndef CVIMAGEWIDGET_H
#define CVIMAGEWIDGET_H

#include <QWidget>
#include <QImage>
#include <QPainter>
#include <QEvent>
#include <QMouseEvent>
#include <opencv2/opencv.hpp>

class CVImageWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CVImageWidget(QWidget *parent = 0);

    QSize sizeHint() const { return _qimage.size(); }
    QSize minimumSizeHint() const { return _qimage.size(); }

    bool getHasROI() const;
    void setHasROI(bool value);

    cv::Rect getRoiRect() const;

signals:

public slots:
    void showImage(const cv::Mat& image);

private:
    bool hasROI = false;
    cv::Rect roiRect;

protected:
    void paintEvent(QPaintEvent* /*event*/);
    bool eventFilter(QObject *object, QEvent *event);

    QImage _qimage;
    cv::Mat _tmp;
};

#endif // CVIMAGEWIDGET_H
