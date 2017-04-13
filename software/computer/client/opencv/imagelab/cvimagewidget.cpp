#include "cvimagewidget.h"

CVImageWidget::CVImageWidget(QWidget *parent) : QWidget(parent)
{
    installEventFilter(this);
}

void CVImageWidget::showImage(const cv::Mat &image)
{
    // Convert the image to the RGB888 format
    switch (image.type())
    {
        case CV_8UC1:
            cvtColor(image, _tmp, CV_GRAY2RGB);
            break;
        case CV_8UC3:
            cvtColor(image, _tmp, CV_BGR2RGB);
            break;
    }

    // QImage needs the data to be stored continuously in memory
    assert(_tmp.isContinuous());
    // Assign OpenCV's image buffer to the QImage. Note that the bytesPerLine parameter
    // (http://qt-project.org/doc/qt-4.8/qimage.html#QImage-6) is 3*width because each pixel
    // has three bytes.
    _qimage = QImage(_tmp.data, _tmp.cols, _tmp.rows, _tmp.cols*3, QImage::Format_RGB888);

    this->setFixedSize(image.cols, image.rows);

    repaint();
}

void CVImageWidget::paintEvent(QPaintEvent *)
{
    // Display the image
    QPainter painter(this);
    painter.drawImage(QPoint(0,0), _qimage);
    painter.end();
}

bool CVImageWidget::eventFilter(QObject *object, QEvent *event)
{
    if (hasROI)
    {
        if(object == this && event->type() == QEvent::MouseButtonPress)
        {
            QMouseEvent *mouseEvent = (QMouseEvent *)event;
            roiRect.x = mouseEvent->pos().x();
            roiRect.y = mouseEvent->pos().y();
            qDebug("Mouse pressed [%d, %d]", mouseEvent->pos().x(), mouseEvent->pos().y());
        }
        if(object == this && event->type() == QEvent::MouseMove)
        {
            QMouseEvent *mouseEvent = (QMouseEvent *)event;
            qDebug("Mouse move [%d, %d]", mouseEvent->pos().x(), mouseEvent->pos().y());
            roiRect.width = mouseEvent->pos().x() - roiRect.x;
            roiRect.height = mouseEvent->pos().y() - roiRect.y;

            QImage cur_qimage = QImage(_tmp.data, _tmp.cols, _tmp.rows, _tmp.cols*3, QImage::Format_RGB888);
            _qimage = cur_qimage;
            QPainter painter(&_qimage);
            QPen myPen(((roiRect.width > 0 && roiRect.height > 0) ? Qt::green : Qt::red), 4, Qt::SolidLine);
            painter.setPen(myPen);
            painter.drawRect(QRect(roiRect.x, roiRect.y, roiRect.width, roiRect.height));
            painter.end();
            repaint();
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}
bool CVImageWidget::getHasROI() const
{
    return hasROI;
}

void CVImageWidget::setHasROI(bool value)
{
    hasROI = value;
}


cv::Rect CVImageWidget::getRoiRect() const
{
    return roiRect;
}
