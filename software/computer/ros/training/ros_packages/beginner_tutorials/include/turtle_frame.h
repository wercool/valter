#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QPoint>
#include <QTimer>

class TurtleFrame : public QFrame
{
	Q_OBJECT

	public:
		TurtleFrame(QWidget* parent = 0, Qt::WindowFlags f = 0);
		~TurtleFrame();
		void positionCallback(const std_msgs::String::ConstPtr& msg);

	protected:
		void paintEvent(QPaintEvent* event);

	private slots:
		void onUpdate();

	private:
		void clear();

		QTimer* update_timer;
		QImage path_image;
		QImage turtleImg;
		float x;
		float y;
};
