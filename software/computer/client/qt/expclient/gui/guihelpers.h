#ifndef GUIHELPERS_H
#define GUIHELPERS_H

#include <QEvent>
#include <mainwindow.h>

class WheelEventFilter: public QObject
{
  public:
  WheelEventFilter():QObject(){}

  ~WheelEventFilter(){}

  bool eventFilter(QObject* object,QEvent* event)
  {
      if(event->type() == QEvent::Wheel)
      {
        return true;
      }
      else
      {
        return QObject::eventFilter(object,event);
      }
  }
};

class GenericEventFilter: public QObject
{
  public:
  Ui::MainWindow *ui;
  QString title;
  int index;
  GenericEventFilter(Ui::MainWindow *ui, QString title, int index):QObject()
  {
      this->ui = ui;
      this->title = title;
      this->index = index;
  }

  ~GenericEventFilter(){}

  bool eventFilter(QObject* widget,QEvent* event)
  {
      if(event->type() == QEvent::Close)
      {
          ui->mainTabWidget->insertTab(index, (QWidget*)widget, title);
          return true;
      }
      else
      {
        return QObject::eventFilter(widget, event);
      }
  }
};

class FrontSonarsFrameEventFilter: public QObject
{
  public:
  Ui::MainWindow *ui;
  FrontSonarsFrameEventFilter(Ui::MainWindow *ui):QObject()
  {
      this->ui = ui;
  }

  ~FrontSonarsFrameEventFilter(){}

  bool eventFilter(QObject* widget, QEvent* event)
  {
      if(event->type() == QEvent::Close)
      {
          ui->platfromLocationP1FronSonarsFrameContainer->addWidget((QWidget*)widget);
          return true;
      }
      else
      {
        return QObject::eventFilter(widget, event);
      }
  }
};

class ACCAndMAGFrameEventFilter: public QObject
{
  public:
  Ui::MainWindow *ui;
  ACCAndMAGFrameEventFilter(Ui::MainWindow *ui):QObject()
  {
      this->ui = ui;
  }

  ~ACCAndMAGFrameEventFilter(){}

  bool eventFilter(QObject* widget, QEvent* event)
  {
      if(event->type() == QEvent::Close)
      {
          ui->accAndMagFrameContainer->addWidget((QWidget*)widget);
          MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->clear();
          return true;
      }
      else if(event->type() == QEvent::Resize)
      {
          MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->setSceneRect(0, 0, ui->accelerometerGraphicsView->width() - 5, ui->accelerometerGraphicsView->height() - 5);
          MainWindow::getInstance()->platformLocationP1AccelerometerGraphicsViewScene->clear();
          MainWindow::getInstance()->platformLocationP1MagnetometerGraphicsViewScene->setSceneRect(0, 0, ui->magnetometerGraphicsView->width() - 5, ui->magnetometerGraphicsView->height() - 5);
          MainWindow::getInstance()->platformLocationP1MagnetometerGraphicsViewScene->clear();
          return true;
      }
      else
      {
        return QObject::eventFilter(widget, event);
      }
  }
};

class CompassFrameEventFilter: public QObject
{
  public:
  Ui::MainWindow *ui;
  CompassFrameEventFilter(Ui::MainWindow *ui):QObject()
  {
      this->ui = ui;
  }

  ~CompassFrameEventFilter(){}

  bool eventFilter(QObject* widget, QEvent* event)
  {
      if(event->type() == QEvent::Close)
      {
          ui->compassFrameContainer->addWidget((QWidget*)widget);
          return true;
      }
      else
      {
        return QObject::eventFilter(widget, event);
      }
  }
};

class InclinometerFrameEventFilter: public QObject
{
  public:
  Ui::MainWindow *ui;
  InclinometerFrameEventFilter(Ui::MainWindow *ui):QObject()
  {
      this->ui = ui;
  }

  ~InclinometerFrameEventFilter(){}

  bool eventFilter(QObject* widget, QEvent* event)
  {
      if(event->type() == QEvent::Close)
      {
          ui->inclinometerFrameContainer->addWidget((QWidget*)widget);
          return true;
      }
      else
      {
        return QObject::eventFilter(widget, event);
      }
  }
};


class IRScanningFrameEventFilter: public QObject
{
  public:
  Ui::MainWindow *ui;
  IRScanningFrameEventFilter(Ui::MainWindow *ui):QObject()
  {
      this->ui = ui;
  }

  ~IRScanningFrameEventFilter(){}

  bool eventFilter(QObject* widget, QEvent* event)
  {
      if(event->type() == QEvent::Close)
      {
          ui->irScanningFrameContainer->addWidget((QWidget*)widget);
          return true;
      }
      else
      {
        return QObject::eventFilter(widget, event);
      }
  }
};

class PlatfromManipulatorFrameEventFilter: public QObject
{
  public:
  Ui::MainWindow *ui;
  PlatfromManipulatorFrameEventFilter(Ui::MainWindow *ui):QObject()
  {
      this->ui = ui;
  }

  ~PlatfromManipulatorFrameEventFilter(){}

  bool eventFilter(QObject* widget, QEvent* event)
  {
      if(event->type() == QEvent::Close)
      {
          ui->manipualtorFrameContainer->addWidget((QWidget*)widget);
          return true;
      }
      else
      {
        return QObject::eventFilter(widget, event);
      }
  }
};

class ValterPlatformMovementControlsEventFilter: public QObject
{
  public:
  Ui::MainWindow *ui;
  ValterPlatformMovementControlsEventFilter(Ui::MainWindow *ui):QObject()
  {
      this->ui = ui;
  }

  ~ValterPlatformMovementControlsEventFilter(){}

  bool eventFilter(QObject* widget, QEvent* event)
  {
      if(event->type() == QEvent::Close)
      {
          ((QWidget*) widget)->layout()->setMargin(0);
          ui->valterPlatformMovementControlsContainer->addWidget((QWidget*)widget);
          return true;
      }
      else
      {
        return QObject::eventFilter(widget, event);
      }
  }
};

#endif // GUIHELPERS_H
