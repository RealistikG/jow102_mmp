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
Mat imgCrop, imgGBlur, imgHSV, imgMask, imgEdges, imgHoughLinesP;

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

    // Crop image
    Rect roi(0,257,640,223);
    imgCrop = cv_ptr->image(roi);

    // Convert image to HSV
    cvtColor(imgCrop, imgHSV, COLOR_BGR2HSV);

    // Trackbars to adjust values for colour mask
    /*namedWindow("Trackbars",(640,200));
    createTrackbar("Hue Min","Trackbars",&hmin,179);
    createTrackbar("Hue Max","Trackbars",&hmax,179);
    createTrackbar("Sat Min","Trackbars",&smin,255);
    createTrackbar("Sat Max","Trackbars",&smax,255);
    createTrackbar("Val Min","Trackbars",&vmin,255);
    createTrackbar("Val Max","Trackbars",&vmax,255);*/

    // Trackbars to adjust values for Canny edge detector
    /*namedWindow("Trackbars2",(640,200));
    createTrackbar("Low Threshold","Trackbars2",&cLowThreshold,100);
    if (cLowThreshold*3>255) cHighThreshold = 255;*/

    // Trackbars to adjust values for HoughLinesP
    /*namedWindow("Trackbars3",(640,200));
    createTrackbar("Threshold","Trackbars3",&hThreshold,100);
    createTrackbar("Min Line Length","Trackbars3",&hMinLineL,100);
    createTrackbar("Max Line Gap","Trackbars3",&hMaxLineG,100);*/

    // Apply colour mask
    Scalar lower (hmin,smin,vmin);
    Scalar upper (hmax,smax,vmax);
    inRange(imgHSV,lower,upper,imgMask);

    // Apply Canny edge detection
    Canny(imgMask,imgEdges,cLowThreshold,cHighThreshold,3);
    imgHoughLinesP = imgCrop.clone();

    // Probabilistic Line Transform ***Code derived from docs.opencv.org tutorial***
    vector<Vec4i> linesP; // Hold results of detection
    HoughLinesP(imgEdges, linesP, 1, CV_PI/180, hThreshold, hMinLineL, hMaxLineG); // Detection
    int xStartR=-1, xEndR=1000, yStartR=-1, yEndR=1000, xStartL=-1, xEndL=-1, yStartL=-1, yEndL=1000;
    // Draw lines
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        Vec4i l = linesP[i];
        line(imgHoughLinesP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
        // Point(xStart, yStart), Point(xEnd, yEnd)

        // *** LINE TRACKING ***
        if(l[1]>l[3]){
            if(l[0]<320 && l[1]>yStartL){
                xStartL=l[0];
                yStartL=l[1];
            } else if(l[0]>320 && l[1]>yStartR){ //l[3]<yEndR
                xStartR=l[0];
                yStartR=l[1];
            }
            if(l[2]<xEndR && l[3]<yEndL){
                xEndL=l[2];
                yEndL=l[3];
            } else if(l[2]>xEndL && l[3]<yEndR){
                xEndR=l[2];
                yEndR=l[3];
            }
        }else{
            if(l[2]<320 && l[3]>yStartL){ //l[1]<yEndL
                xStartL=l[2];
                yStartL=l[3];
            } else if(l[2]>320 && l[3]>yStartR){ //l[1]<yEndR
                xStartR=l[2];
                yStartR=l[3];
            }
            if(l[0]<xEndR && l[1]<yEndL){
                xEndL=l[0];
                yEndL=l[1];
            } else if(l[0]>xEndL && l[1]<yEndR){
                xEndR=l[0];
                yEndR=l[1];
            }
        }

        // Rline -> if xStartR < 320 && yStartR > prevYStartR;
        // Lline -> if xEndL > 320 && yEndL > prevYEndL;
        // Draws down -> up on L; up -> down on R;
    }

    // draw circles of radius 10 around the xy point
    bool lBool = false, rBool = false;
    if(xStartL != -1 && yStartL != -1){
        circle(imgHoughLinesP, Point(xStartL, yStartL), 10, CV_RGB(0,0,255));
        circle(imgHoughLinesP, Point(xEndL, yEndL), 10, CV_RGB(0,0,255));
        lBool = true;
    }
    if(xStartR!=-1 && yStartR!=-1){
        circle(imgHoughLinesP, Point(xStartR, yStartR), 10, CV_RGB(0,0,255));
        circle(imgHoughLinesP, Point(xEndR, yEndR), 10, CV_RGB(0,0,255));
        rBool = true;
    }
    int xStartC=-1, yStartC=-1, xEndC=-1, yEndC=-1;
    if(lBool && rBool){
        xStartC=(xStartR+xStartL)/2;
        yStartC=(yStartR+yStartL)/2;
        xEndC=(xEndR+xEndL)/2;
        yEndC=(yEndR+yEndL)/2;
        circle(imgHoughLinesP, Point(xStartC, yStartC), 10, CV_RGB(0,255,0));
        circle(imgHoughLinesP, Point(xEndC, yEndC), 10, CV_RGB(0,255,0));
        line(imgHoughLinesP, Point(xStartC, yStartC), Point(xEndC, yEndC), Scalar(0,255,0), 3, LINE_AA);

        xTrack=(xStartC+xEndC)/2;
        yTrack=(yStartC+yEndC)/2;
    } else {
        xTrack = 320;
        yTrack = 240;
    }

    // Update GUI Windows
    imshow(OPENCV_WINDOW, cv_ptr->image);
    //imshow("HSV",imgHSV);
    //imshow("Mask",imgMask);
    //imshow("Edges",imgEdges);
    imshow("HoughLinesP",imgHoughLinesP);
    waitKey(25);
}

void drive(){
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
}

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
    int startTime = Time::now().toSec(), lastUpdateTime = startTime;
    while(true){
        int timeNow = Time::now().toSec();
        // Spin ros once every second
        if (timeNow-lastUpdateTime > 1){
            ROS_INFO("Updating");
            spinOnce();
            drive();
            lastUpdateTime = Time::now().toSec();
        }
        // Break loop and end program after x seconds
        if(timeNow-startTime > 60){
            ROS_INFO("60s Elapsed: End Program");
            break;
        }
    }
    return 0;
}


