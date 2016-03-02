#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QPoint>
#include <QTimer>
#include <sensor_msgs/Joy.h>

class TurtleFrame : public QFrame
{
	Q_OBJECT

	public:
		TurtleFrame(QWidget* parent = 0, Qt::WindowFlags f = 0);
		~TurtleFrame();
		void positionCallback(const std_msgs::String::ConstPtr& msg);
		void joyPositionCallback(const sensor_msgs::Joy::ConstPtr& msg);

	protected:
		void paintEvent(QPaintEvent* event);

	private slots:
		void onUpdate();
		float dx;
		float dy;

	private:
		void clear();

		QTimer* update_timer;
		QImage path_image;
		QImage turtleImg;
		QImage ballImg;
		float x;
		float y;
		float x1;
		float y1;
};
