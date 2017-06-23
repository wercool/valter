#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include<stdio.h> //printf
#include<string.h>    //strlen
#include<string>  //string
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent
#include<math.h>

static const std::string OPENCV_WINDOW = "Image window";
bool showWindow = false;

std::string face_cascade_name = "/home/maska/git/valter/software/ros/workspaces/master/src/valter/src/haarcascade_frontalface_alt.xml";

inline std::string format(const char* fmt, ...)
{
    int size = 512;
    char* buffer = 0;
    buffer = new char[size];
    va_list vl;
    va_start(vl, fmt);
    int nsize = vsnprintf(buffer, size, fmt, vl);
    if(size<=nsize){ //fail delete buffer and try again
        delete[] buffer;
        buffer = 0;
        buffer = new char[nsize+1]; //+1 for /0
        nsize = vsnprintf(buffer, size, fmt, vl);
    }
    std::string ret(buffer);
    va_end(vl);
    delete[] buffer;
    return ret;
}

class FaceTracker
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    cv::CascadeClassifier face_cascade;
    int prevCenterX, prevCenterY;

public:
    FaceTracker(): it(nh)
    {
        prevCenterX = prevCenterY = -1;
        // Subscrive to input video feed and publish output video feed
        image_sub = it.subscribe("/kinect1/rgb/image_rect_color_drop", 1, &FaceTracker::imageCb, this);
        image_pub = it.advertise("/face_detector/output_video", 1);

        if(!face_cascade.load(face_cascade_name)){ printf("(!) Error loading face cascade\n"); };

        if (showWindow)
        {
            cv::namedWindow(OPENCV_WINDOW);
        }
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

        detectAndDisplay(cv_ptr->image);

        // Update GUI Window
        if (showWindow)
        {
            cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        }

        // Output modified video stream
        image_pub.publish(cv_ptr->toImageMsg());
    }

    void detectAndDisplay(cv::Mat frame)
    {
        std::vector<cv::Rect> faces;
        cv::Mat frame_gray;

        cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
        cv::equalizeHist(frame_gray, frame_gray);

        //-- Detect faces
        face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
/*
        for(size_t i = 0; i < faces.size(); i++)
        {
            cv::Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);
            cv::ellipse(frame, center, cv::Size(faces[i].width * 0.75, faces[i].height * 0.75), 0, 0, 360, cv::Scalar(0, 255, 0), 2, 4, 0);
        }
*/
        if (faces.size() > 0)
        {
            cv::Point center(faces[0].x + faces[0].width * 0.5, faces[0].y + faces[0].height * 0.5);
            cv::ellipse(frame, center, cv::Size(faces[0].width * 0.5, faces[0].height * 0.5), 0, 0, 360, cv::Scalar(0, 255, 0), 2, 4, 0);

            if (prevCenterX == -1 && prevCenterY == -1)
            {
                prevCenterX = center.x;
                prevCenterY = center.y;
            }

            int distanceX = pow((center.x - prevCenterX), 2);
            int distanceY = pow((center.y - prevCenterY), 2);

            double distance = sqrt(distanceX + distanceY);

            ROS_INFO("Face X: %d, Y: %d, D: %.2f %s", center.x, center.y, distance, (distance > 20) ? "IGNORED" : "");

            prevCenterX = center.x;
            prevCenterY = center.y;

            if (distance < 20)
            {
                double degYawPerPixel = 45.0 / (frame_gray.cols / 2);
                int yaw = round((center.x - frame_gray.cols / 2) * degYawPerPixel) ;
                ROS_INFO("Yaw: %d", yaw);

                int sock = socket(AF_INET , SOCK_STREAM , 0);
                struct sockaddr_in server;
                server.sin_addr.s_addr = inet_addr("192.168.101.101");
                server.sin_family = AF_INET;
                server.sin_port = htons(55555);

                //Connect to remote server
                if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
                {
                    perror("expclient connection failed. Error");
                }
                else
                {
                    //Head Yaw
                    int returnedBytes;
                    std::string headYawTask = format("T_BCP1_SetHeadYawPositionTask_%d", yaw);
                    char buffer[1024];

                    for (int i = 0; i < 1024; i++)
                    {
                        buffer[i] = '\0';
                    }

                    returnedBytes = send(sock , headYawTask.c_str() , strlen(headYawTask.c_str()) , 0);

                    if(returnedBytes < 0)
                    {
                        perror("Send failed. Error");
                    }
                }

            }
        }
    }

};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "face_tracker");
    ros::NodeHandle nh;
    if (argc > 1)
    {
        if (std::string(argv[1]) == "show_window")
        {
            showWindow = true;
            ROS_INFO("SHOW WINDOW");
        }
    }
    if (argc > 2)
    {
        face_cascade_name = std::string(argv[2]);
    }
    FaceTracker ft;
    ros::spin();
    return 0;
}
