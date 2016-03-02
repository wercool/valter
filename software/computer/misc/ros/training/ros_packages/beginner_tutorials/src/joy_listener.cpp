#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include <stdlib.h>

#include <sensor_msgs/Joy.h>

#include "turtle_frame.h"

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

#include <QApplication>

#define DEFAULT_BG_R 0x50
#define DEFAULT_BG_G 0x50
#define DEFAULT_BG_B 0x50

TurtleFrame::TurtleFrame(QWidget* parent, Qt::WindowFlags fm)
:QFrame(parent, fm)
//,path_image(500, 500, QImage::Format_ARGB32)
{
	//setFixedSize(500, 500);
	resize(500,600);
	setStyleSheet("background-color:grey;");
	setWindowTitle("TurtleSim");

	x = 200;
	y = 250;

	x1 = 300;
	y1 = 250;

	update_timer = new QTimer(this);
	update_timer->setInterval(10);
	update_timer->start();

	connect(update_timer, SIGNAL(timeout()), this, SLOT(onUpdate()));

	std::string turtle_img_path = ros::package::getPath("beginner_tutorials") + "/images/ball.png";
	std::string ball_img_path = ros::package::getPath("beginner_tutorials") + "/images/ball1.png";

	ROS_INFO("START");
	ROS_INFO("image1 path: %s", turtle_img_path.c_str());
	ROS_INFO("image2 path: %s", ball_img_path.c_str());

	QString image_path = turtle_img_path.c_str();
	turtleImg.load(image_path);

	ROS_INFO("image height: %d", turtleImg.height());

	QString image1_path = ball_img_path.c_str();
	ballImg.load(image1_path);

	ROS_INFO("image height: %d", ballImg.height());


	x -= turtleImg.width() / 2;
	y -= turtleImg.height() / 2;

	x1 -= ballImg.width() / 2;
	y1 -= ballImg.height() / 2;

	clear();
}
TurtleFrame::~TurtleFrame()
{
	delete update_timer;
}
void TurtleFrame::onUpdate()
{
	ros::spinOnce();

	if (!ros::ok())
	{
		close();
	}

	x1 -= dx;
	y1 -= dy;

	update();
}
void TurtleFrame::clear()
{
	int r = DEFAULT_BG_R;
	int g = DEFAULT_BG_G;
	int b = DEFAULT_BG_B;
	path_image.fill(qRgb(r, g, b));
	update();
}
void TurtleFrame::paintEvent(QPaintEvent* event)
{
	QPainter painter(this);
	painter.drawImage(QPoint(0, 0), path_image);
	painter.drawImage(QPoint(x, y), turtleImg);
	painter.drawImage(QPoint(x1, y1), ballImg);
}
void TurtleFrame::positionCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	std::vector<std::string> tokens;
	std::string msgStr =  msg->data.c_str();
	boost::split(tokens, msgStr, boost::is_any_of(","));

	int FB = 508;
	int LR = 500;

	std::string token;
	std::vector<std::string> tokenvalues;

	token = tokens.at(0);
	tokenvalues.clear();
	boost::split(tokenvalues, token, boost::is_any_of(":"));
	token = tokenvalues.at(1);
	FB = FB - atoi(token.c_str());

	token = tokens.at(1);
	tokenvalues.clear();
	boost::split(tokenvalues, token, boost::is_any_of(":"));
	token = tokenvalues.at(1);
	LR = LR - atoi(token.c_str());

	if (abs(LR) > 10)
	{
		x += LR / 50;
	}
	if (abs(FB) > 10)
	{
		y += FB / 50;
	}

	//std::cout << FB << std::endl;
}

void TurtleFrame::joyPositionCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	/*
	for (unsigned i = 0; i < msg->axes.size(); ++i)
	{
		ROS_INFO("Axis %d is now at position %f", i, msg->axes[i]);
	}
	*/

	float FB = msg->axes[1];
	float LR = msg->axes[0];

	dx = LR * 2;
	dy = FB * 2;

	ROS_INFO("dx: %f", dx);
	ROS_INFO("dy: %f", dy);
}

class TurtleApp : public QApplication
{
	private:
		TurtleFrame frame;
	public:
		TurtleApp(int& argc, char** argv):QApplication(argc, argv)
		{

		}
		int exec()
		{
			frame.show();
			return QApplication::exec();
		}
		TurtleFrame* getTurtleFrame()
		{
			return &frame;
		}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "listener", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	TurtleApp app(argc, argv);

	ros::Subscriber sub1 = nh.subscribe("chatter", 1000, &TurtleFrame::positionCallback, app.getTurtleFrame());
	ros::Subscriber sub2 = nh.subscribe("joy", 1000, &TurtleFrame::joyPositionCallback, app.getTurtleFrame());

	return app.exec();
}















///**
// * This tutorial demonstrates simple receipt of messages over the ROS system.
// */
//void chatterCallback(const std_msgs::String::ConstPtr& msg)
//{
//  ROS_INFO("I heard: [%s]", msg->data.c_str());
//}
//
//int main(int argc, char **argv)
//{
//  /**
//   * The ros::init() function needs to see argc and argv so that it can perform
//   * any ROS arguments and name remapping that were provided at the command line.
//   * For programmatic remappings you can use a different version of init() which takes
//   * remappings directly, but for most command-line programs, passing argc and argv is
//   * the easiest way to do it.  The third argument to init() is the name of the node.
//   *
//   * You must call one of the versions of ros::init() before using any other
//   * part of the ROS system.
//   */
//  ros::init(argc, argv, "listener");
//
//  /**
//   * NodeHandle is the main access point to communications with the ROS system.
//   * The first NodeHandle constructed will fully initialize this node, and the last
//   * NodeHandle destructed will close down the node.
//   */
//  ros::NodeHandle n;
//
//  /**
//   * The subscribe() call is how you tell ROS that you want to receive messages
//   * on a given topic.  This invokes a call to the ROS
//   * master node, which keeps a registry of who is publishing and who
//   * is subscribing.  Messages are passed to a callback function, here
//   * called chatterCallback.  subscribe() returns a Subscriber object that you
//   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
//   * object go out of scope, this callback will automatically be unsubscribed from
//   * this topic.
//   *
//   * The second parameter to the subscribe() function is the size of the message
//   * queue.  If messages are arriving faster than they are being processed, this
//   * is the number of messages that will be buffered up before beginning to throw
//   * away the oldest ones.
//   */
//  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
//
//  /**
//   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
//   * callbacks will be called from within this thread (the main one).  ros::spin()
//   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
//   */
//  ros::spin();
//
//  return 0;
//}
