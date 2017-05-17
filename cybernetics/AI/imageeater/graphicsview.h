#ifndef GRAPHICSVIEW_H
#define GRAPHICSVIEW_H

#include <QGraphicsView>
#include <QWheelEvent>

#include <qmath.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

class GraphicsView : public QGraphicsView
{
public:
    GraphicsView(QWidget *parent = 0);

    void zoomIn(int level);
    void zoomOut(int level);

    void setupMatrix();

protected:
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    int zoomLevel = 0.0;
    DragMode dragMode = DragMode::RubberBandDrag;
};

#endif // GRAPHICSVIEW_H
