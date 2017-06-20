#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class FaceTracker
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

public:
    FaceTracker(): it(nh)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub = it.subscribe("/camera/rgb/image_rect_color", 1, &FaceTracker::imageCb, this);
        image_pub = it.advertise("/face_detector/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~FaceTracker()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw an example circle on the video stream
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        {
            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
        }

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        image_pub.publish(cv_ptr->toImageMsg());
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "face_tracker");
    FaceTracker ft;
    ros::spin();
    return 0;
}
