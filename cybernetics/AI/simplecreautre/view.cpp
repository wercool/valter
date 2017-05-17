#include "view.h"

#ifndef QT_NO_OPENGL
#include <QtOpenGL>
#else
#include <QtWidgets>
#endif
#include <qmath.h>

//#define QT_NO_OPENGL

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

void GraphicsView::drawBackground(QPainter *painter, const QRectF &rect)
{
    Q_UNUSED(rect);
    painter->drawPixmap(QRect(0, 0, view->getEnvPixmap().width(), view->getEnvPixmap().height()), view->getEnvPixmap());
}
#endif

View::View(QWidget *parent): QFrame(parent)
{
    setFrameStyle(Sunken | StyledPanel);

    graphicsView = new GraphicsView(this);
    graphicsView->setRenderHint(QPainter::Antialiasing, true);
    graphicsView->setDragMode(QGraphicsView::RubberBandDrag);
    graphicsView->setOptimizationFlags(QGraphicsView::DontAdjustForAntialiasing | QGraphicsView::DontSavePainterState);
    graphicsView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    graphicsView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    graphicsView->setCacheMode(QGraphicsView::CacheBackground);

    #ifndef QT_NO_OPENGL
        if (QGLFormat::hasOpenGL())
        {
            graphicsView->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
        }
        else
        {
            graphicsView->setViewport(new QWidget());
        }
    #else
        graphicsView->setViewport(new QWidget);
    #endif

    int size = style()->pixelMetric(QStyle::PM_ToolBarIconSize);
    QSize iconSize(size, size);

    QToolButton *zoomInIcon = new QToolButton;
    zoomInIcon->setAutoRepeat(true);
    zoomInIcon->setAutoRepeatInterval(10);
    zoomInIcon->setAutoRepeatDelay(0);
    zoomInIcon->setIcon(QPixmap(":/zoomin.png"));
    zoomInIcon->setIconSize(iconSize);
    QToolButton *zoomOutIcon = new QToolButton;
    zoomOutIcon->setAutoRepeat(true);
    zoomOutIcon->setAutoRepeatInterval(10);
    zoomOutIcon->setAutoRepeatDelay(0);
    zoomOutIcon->setIcon(QPixmap(":/zoomout.png"));
    zoomOutIcon->setIconSize(iconSize);
    zoomSlider = new QSlider;
    zoomSlider->setMinimum(-300);
    zoomSlider->setMaximum(400);
    zoomSlider->setValue(100);
    zoomSlider->setTickPosition(QSlider::TicksRight);

    // Zoom slider layout
    QVBoxLayout *zoomSliderLayout = new QVBoxLayout;
    zoomSliderLayout->addWidget(zoomInIcon);
    zoomSliderLayout->addWidget(zoomSlider);
    zoomSliderLayout->addWidget(zoomOutIcon);

    selectModeButton = new QToolButton;
    selectModeButton->setText(tr("Select"));
    selectModeButton->setCheckable(true);
    selectModeButton->setChecked(true);
    dragModeButton = new QToolButton;
    dragModeButton->setText(tr("Drag"));
    dragModeButton->setCheckable(true);
    dragModeButton->setChecked(false);

    QButtonGroup *pointerModeGroup = new QButtonGroup(this);
    pointerModeGroup->setExclusive(true);
    pointerModeGroup->addButton(selectModeButton);
    pointerModeGroup->addButton(dragModeButton);

    QGridLayout *graphicsLayout = new QGridLayout;
    QHBoxLayout *toolBoxLayout = new QHBoxLayout;
    toolBoxLayout->addWidget(selectModeButton);
    toolBoxLayout->addWidget(dragModeButton);

    graphicsLayout->addLayout(toolBoxLayout, 1, 0);
    graphicsLayout->addWidget(graphicsView, 2, 0);
    graphicsLayout->addLayout(zoomSliderLayout, 2, 1);
    setLayout(graphicsLayout);

    connect(selectModeButton, SIGNAL(toggled(bool)), this, SLOT(togglePointerMode()));
    connect(dragModeButton, SIGNAL(toggled(bool)), this, SLOT(togglePointerMode()));
    connect(zoomSlider, SIGNAL(valueChanged(int)), this, SLOT(setupMatrix()));
    connect(zoomInIcon, SIGNAL(clicked()), this, SLOT(zoomIn()));
    connect(zoomOutIcon, SIGNAL(clicked()), this, SLOT(zoomOut()));

    setupMatrix();
}

QGraphicsView *View::grpahicsView() const
{
    return static_cast<QGraphicsView *>(graphicsView);
}

void View::zoomIn(int level)
{
    zoomSlider->setValue(zoomSlider->value() + level);
}

void View::zoomOut(int level)
{
    zoomSlider->setValue(zoomSlider->value() - level);
}

void View::setupMatrix()
{
    qreal scale = qPow(qreal(2), (zoomSlider->value() - 100) / qreal(50));

    QMatrix matrix;
    matrix.scale(scale, scale);

    graphicsView->setMatrix(matrix);
}

void View::togglePointerMode()
{
    graphicsView->setDragMode(selectModeButton->isChecked()
                              ? QGraphicsView::RubberBandDrag
                              : QGraphicsView::ScrollHandDrag);
//    graphicsView->setInteractive(selectModeButton->isChecked());
}

cv::Mat View::getEnvMapMat() const
{
    return envMapMat;
}

void View::setEnvMapMat(cv::Mat value)
{
    envMapMat = value;
    setEnvPixmap(cvMatToQPixmap(envMapMat));
    getGraphicsView()->resetCachedContent();
}

void View::updateEnvMap()
{
    setEnvPixmap(cvMatToQPixmap(envMapMat));
    getGraphicsView()->resetCachedContent();
}

GraphicsView *View::getGraphicsView() const
{
    return graphicsView;
}

void View::setGraphicsView(GraphicsView *value)
{
    graphicsView = value;
}

QPixmap View::getEnvPixmap() const
{
    return envPixmap;
}

void View::setEnvPixmap(QPixmap value)
{
    envPixmap = value;
}

