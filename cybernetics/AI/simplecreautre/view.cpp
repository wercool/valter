#include "view.h"

#ifndef QT_NO_OPENGL
#include <QtOpenGL>
#else
#include <QtWidgets>
#endif
#include <qmath.h>

#ifndef QT_NO_WHEELEVENT
void GraphicsView::wheelEvent(QWheelEvent *e)
{
    if (e->modifiers() & Qt::ControlModifier)
    {
        if (e->delta() > 0)
        {
            view->zoomIn(6);
        }
        else
        {
            view->zoomOut(6);
        }
        e->accept();
    }
    else
    {
        QGraphicsView::wheelEvent(e);
    }
}
#endif

View::View(const QString &name, QWidget *parent): QFrame(parent)
{
    setFrameStyle(Sunken | StyledPanel);

    graphicsView = new GraphicsView(this);
    graphicsView->setRenderHint(QPainter::Antialiasing, false);
    graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
    graphicsView->setOptimizationFlags(QGraphicsView::DontSavePainterState);
    graphicsView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    graphicsView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(graphicsView, 1, 0);

    setLayout(layout);

    setupMatrix();
}

QGraphicsView *View::view() const
{
    return static_cast<QGraphicsView *>(graphicsView);
}

void View::zoomIn(int level)
{
    scale += level;
    setupMatrix();
}

void View::zoomOut(int level)
{
    scale -= level;
    setupMatrix();
}

void View::resetView()
{

}

void View::setupMatrix()
{
    qreal scaleVal = qPow(qreal(2), ((qreal)scale - (qreal)250) / (qreal)50);

    QMatrix matrix;
    matrix.scale(scaleVal, scaleVal);
    matrix.rotate(0.0);

    qDebug("%.2f", scaleVal);

    graphicsView->setMatrix(matrix);
}

void View::togglePointerMode()
{

}

void View::toggleOpenGL()
{

}

void View::toggleAntialiasing()
{

}

