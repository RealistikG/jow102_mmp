// include ros lib
#include <ros/ros.h>

// include opencv2 lib
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
Mat imgHSV, imgMask, imgEdges, imgHoughLines;
int hmin = 0, smin = 0, vmin = 255;
int hmax = 0, smax = 0, vmax = 255;

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    //ros::NodeHandle imageProc;
    //image_transport::ImageTransport it2(imageProc);
    //image_transport::Publisher pub = it2.advertise("/image_converter/output_video", 1);
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV); // Converts image to HSV

    /*namedWindow("Trackbars",(640,200));
    createTrackbar("Hue Min","Trackbars",&hmin,179);
    createTrackbar("Hue Max","Trackbars",&hmax,179);
    createTrackbar("Sat Min","Trackbars",&smin,255);
    createTrackbar("Sat Max","Trackbars",&smax,255);
    createTrackbar("Val Min","Trackbars",&vmin,255);
    createTrackbar("Val Max","Trackbars",&vmax,255);*/

    Scalar lower (hmin,smin,vmin);
    Scalar upper (hmax,smax,vmax);
    inRange(imgHSV,lower,upper,imgMask);    // Applies mask

    Canny(imgMask,imgEdges,200,255);   // Edge detection

    // Probabilistic Line Transform
    vector<Vec4i> linesP; // Hold results of detection
    HoughLinesP(imgEdges, linesP, 1, CV_PI/180, 50, 50, 10 ); // Detection
    // Draw lines
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        line(imgEdges, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
    }

    // Update GUI Window
    imshow(OPENCV_WINDOW, cv_ptr->image);
    //imshow("HSV",imgHSV);
    //imshow("Mask",imgMask);
    imshow("Edges",imgEdges);
    //imshow("Hough Lines",imgHoughLines);
    waitKey(25);

    //pub.publish(cv_ptr->toImageMsg());
    //imwrite("/impacs/jow102/catkin_ws/src/jow102_mmp/test_img.jpg", cv_ptr->image);
    //imwrite("/impacs/jow102/catkin_ws/src/jow102_mmp/test_img_hsv.jpg", imgHSV); //save hsv test image
}

int main(int argc, char **argv) {
    // Initialize the ROS system.
    ros::init(argc, argv, "test");

    // Establish this program as a ROS node.
    ros::NodeHandle nh;

    // Subscribe to input image topic using image transport.
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, image_cb);
    //image_transport::Publisher pub = it.advertise("/image_converter/output_video", 1);
    namedWindow(OPENCV_WINDOW);

    //ros::spinOnce();
    while(true){
        ros::spinOnce();
        //break;
    }
    return 0;
}



