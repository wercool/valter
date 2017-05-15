#ifndef VIEW_H
#define VIEW_H

#include <QFrame>
#include <QGraphicsView>
#include <QSlider>
#include <QToolButton>

#include "utils/opencv-qt/opencvMatToQPixmap.h"

class View;

class GraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    GraphicsView(View *v) : QGraphicsView(), view(v) { }

protected:
#ifndef QT_NO_WHEELEVENT
    void wheelEvent(QWheelEvent *) override;
#endif

private:
    View *view;

    // QGraphicsView interface
protected:
    void drawBackground(QPainter *painter, const QRectF &rect);
};

class View : public QFrame
{
    Q_OBJECT
public:
    explicit View(QWidget *parent = 0);

    QGraphicsView *grpahicsView() const;

    QPixmap getEnvPixmap() const;
    void setEnvPixmap(QPixmap value);

    GraphicsView *getGraphicsView() const;
    void setGraphicsView(GraphicsView *value);

    cv::Mat getEnvMapMat() const;
    void setEnvMapMat(cv::Mat value);

    void updateEnvMap();

public slots:
    void zoomIn(int level = 1);
    void zoomOut(int level = 1);

private slots:
    void setupMatrix();
    void togglePointerMode();

private:
    GraphicsView *graphicsView;
    QToolButton *selectModeButton;
    QToolButton *dragModeButton;
    QSlider *zoomSlider;

    cv::Mat envMapMat;
    QPixmap envPixmap;
};

#endif // VIEW_H
