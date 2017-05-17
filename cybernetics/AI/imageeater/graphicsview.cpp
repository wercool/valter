#include "graphicsview.h"

GraphicsView::GraphicsView(QWidget *parent): QGraphicsView()
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

