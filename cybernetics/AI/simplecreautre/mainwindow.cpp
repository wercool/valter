#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "view.h"
#include "creature.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    populateScene();

    View *view = new View("Simple Creature Graphics");
    view->view()->setScene(scene);
//    ui->graphicsView->setScene(scene);

    ui->verticalLayoutGraphics->addWidget(view);

    setWindowTitle(tr("Simple Creature Evolution"));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::populateScene()
{
    scene = new QGraphicsScene(this);

    for (int x = 20; x < 1000; x += 40)
    {
        for (int y = 20; y < 1000; y += 40)
        {
            QGraphicsItem *item = new Creature(x, y, QColor(0, 255, 0, 127));
            item->setPos(QPointF(x, y));
            scene->addItem(item);
        }
    }
}
