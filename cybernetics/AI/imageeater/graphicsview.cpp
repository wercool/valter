#include "graphicsview.h"

GraphicsView::GraphicsView(QWidget *parent)
{
    Q_UNUSED(parent);
}

void GraphicsView::wheelEvent(QWheelEvent *event)
{
    if (event->modifiers() & Qt::ControlModifier)
    {
        if (event->delta() > 0)
        {
            zoomIn(6);
        }
        else
        {
            zoomOut(6);
        }
        event->accept();
    }
    else
    {
        QGraphicsView::wheelEvent(event);
    }
}

void GraphicsView::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        if (event->modifiers() & Qt::ControlModifier)
        {
            setDragMode(QGraphicsView::ScrollHandDrag);
        }
    }
    QGraphicsView::mousePressEvent(event);
}

void GraphicsView::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        setDragMode(QGraphicsView::RubberBandDrag);
    }
    QGraphicsView::mousePressEvent(event);
}

void GraphicsView::zoomIn(int level)
{
    zoomLevel += level;
    setupMatrix();
}

void GraphicsView::zoomOut(int level)
{
    zoomLevel -= level;
    setupMatrix();
}

void GraphicsView::setupMatrix()
{
    qreal scale = qPow(qreal(2), zoomLevel / qreal(50));

    QMatrix matrix;
    matrix.scale(scale, scale);

    setMatrix(matrix);
}

