// include ros lib
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// include opencv2 lib
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
using namespace ros;

// Image Windows
static const std::string OPENCV_WINDOW = "Image window";
Mat img, imgCrop, imgHSV, imgMask, imgEdges, imgHoughLinesP;

// Hue, Sat, Value min & max values for colour mask
int hmin = 0, smin = 0, vmin = 255;
int hmax = 0, smax = 0, vmax = 255;

// Canny edge detection values
int cLowThreshold = 50, cHighThreshold = 150;

// HoughLinesP values
int hThreshold = 15, hMinLineL = 10, hMaxLineG = 90;

int xTrack, yTrack;

void image_cb(const sensor_msgs::ImageConstPtr& msg)
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

    img = cv_ptr->image.clone();
    imshow(OPENCV_WINDOW, cv_ptr->image);
    //imshow("img",img);
    waitKey(25);
}

void imageProc(){
    imshow("img", img);
    waitKey(25);
}

/*void drive(){
    NodeHandle driveNh;
    Publisher pub = driveNh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    geometry_msgs::Twist values;

    int deadzone = 100;
    values.linear.x = 0.2;
    if (xTrack>320+deadzone){
        values.angular.z = -0.2;
    } else if (xTrack<320-deadzone){
        values.angular.z = 0.2;
    } else values.angular.z = 0;

    pub.publish(values);
}*/

int main(int argc, char **argv) {
    // Initialize the ROS system.
    init(argc, argv, "test");

    // Establish this program as a ROS node.
    NodeHandle mainNh;

    // Subscribe to input image topic using image transport.
    image_transport::ImageTransport it(mainNh);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, image_cb);
    namedWindow(OPENCV_WINDOW);

    spinOnce();
    int startTime = Time::now().toSec(), currentTime = startTime;
    while(currentTime-startTime<1){
        currentTime = Time::now().toSec();
    }
    Rate r(10); // 10Hz
    while(ros::ok){
        spinOnce();
        r.sleep();
        imageProc();
    }
    return 0;
}


