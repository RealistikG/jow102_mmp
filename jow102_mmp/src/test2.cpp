// Include ros lib
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Include opencv2 lib
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Include misc
#include <pthread.h>
#include <iostream>

using namespace cv;
using namespace std;
using namespace ros;

// Image Windows
Mat img, imgCrop, imgHSV, imgMask, imgEdges, imgHoughLinesP;

// Canny edge detection values
int cLowThreshold = 50, cHighThreshold = 150;

int xTrack = 320;

bool wait = true;

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

    //cout << "CB TEST" << endl;
    //ROS_INFO("CB TEST");
    img = cv_ptr->image.clone();
    imshow("Image", cv_ptr->image);
    waitKey(10);
}

void *imageProc(void *paramID){
    long tid;
    tid = (long)paramID;
    cout << "Thread ID: " << tid << endl;

    Rate rate(10);

    // Short loop for testing
    int startTime = Time::now().toSec(), currentTime = startTime;
    while(currentTime-startTime<3){
        currentTime = Time::now().toSec();
        rate.sleep();
    }

    wait = false;

    // Main image proc loop
    while(ros::ok()){
        //out << "ImageProc TEST" << endl;

        // Crop image
        Rect roi(0,257,640,223);
        imgCrop = img(roi);

        // Convert image to HSV
        cvtColor(imgCrop, imgHSV, COLOR_BGR2HSV);

        // Apply colour mask
        Scalar lower (0,0,255);
        Scalar upper (0,0,255);
        inRange(imgHSV,lower,upper,imgMask);

        // Apply Canny edge detection
        Canny(imgMask,imgEdges, 50, 150, 3);
        imgHoughLinesP = imgCrop.clone();

        // Probabilistic Line Transform ***Code derived from docs.opencv.org tutorial***
        // Vector to hold results of HoughLinesP detection
        vector<Vec4i> linesP;
        HoughLinesP(imgEdges, linesP, 1, CV_PI/180, 15, 10, 90); // Detection

        // Left Line
        Vec4i left;
        left[0] = -1;
        left[1] = -1;
        left[2] = -1;
        left[3] = 1000;

        // Right Line
        Vec4i right;
        right[0] = -1;
        right[1] = -1;
        right[2] = 1000;
        right[3] = 1000;

        // Loop through vector, select most appropriate lines for lane tracking
        for( size_t i = 0; i < linesP.size(); i++ )
        {
            // *** LINE TRACKING ***
            Vec4i x = linesP[i];
            if(x[1]>x[3]){
                if(x[0]<320 && x[1]>left[1]){
                    left[0]=x[0];
                    left[1]=x[1];
                } else if(x[0]>320 && x[1]>right[1]){
                    right[0]=x[0];
                    right[1]=x[1];
                }
                if(x[2]<(right[2]-10) && x[3]<left[3]){
                    left[2]=x[2];
                    left[3]=x[3];
                } else if(x[2]>(left[2]+10) && x[3]<right[3]){
                    right[2]=x[2];
                    right[3]=x[3];
                }
            }else{
                if(x[2]<320 && x[3]>left[1]){
                    left[0]=x[2];
                    left[1]=x[3];
                } else if(x[2]>320 && x[3]>right[1]){
                    right[0]=x[2];
                    right[1]=x[3];
                }
                if(x[0]<(right[2]-10) && x[1]<left[3]){
                    left[2]=x[0];
                    left[3]=x[1];
                } else if(x[0]>(left[2]+10) && x[1]<right[3]){
                    right[2]=x[0];
                    right[3]=x[1];
                }
            }
        }

        // Draw detected lane lines
        bool lBool = false, rBool = false;
        if(left[0] != -1 && left[1] != -1){
            line(imgHoughLinesP, Point(left[0], left[1]), Point(left[2], left[3]), Scalar(255,0,0), 3, LINE_AA);
            lBool = true;
        }
        if(right[0] != -1 && right[1] != -1){
            line(imgHoughLinesP, Point(right[0], right[1]), Point(right[2], right[3]), Scalar(255,0,0), 3, LINE_AA);
            rBool = true;
        }

        // Centre line
        Vec4i centre;
        if(lBool && rBool){
            centre[0] = (right[0]+left[0])/2;
            centre[1] = (right[1]+left[1])/2;
            centre[2] = (right[2]+left[2])/2;
            centre[3] = (right[3]+left[3])/2;
            line(imgHoughLinesP, Point(centre[0], centre[1]), Point(centre[2], centre[3]), Scalar(0,255,0), 3, LINE_AA);
            xTrack = (centre[0]+centre[2])/2;
        } else {
            xTrack = 320;
        }

        // Update GUI Window
        imshow("HoughLinesP",imgHoughLinesP);
        waitKey(10);

        rate.sleep();
    }
    pthread_exit(NULL);
}

void drive(Publisher pub){
    if(wait){
        return;
    }

    geometry_msgs::Twist values;
    int deadzone = 35;
    values.linear.x = 0.2;
    if (xTrack>320+deadzone){
        values.angular.z = -0.1;
    } else if (xTrack<320-deadzone){
        values.angular.z = 0.1;
    } else {
        values.angular.z = 0;
    }

    pub.publish(values);
}

int main(int argc, char **argv) {
    // Initialize ROS
    init(argc, argv, "jow102_mmp");
    NodeHandle mainNh;

    // Subscribe to input image topic using image transport
    image_transport::ImageTransport it(mainNh);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, image_cb);

    // Publisher for driving
    Publisher pub = mainNh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    Rate rate(10);
    spinOnce();
    rate.sleep();

    // Create new thread
    pthread_t thread;
    int rc = pthread_create(&thread, NULL, imageProc, (void *)1);
    if (rc) {
        cout << "Error:unable to create thread," << rc << endl;
        exit(-1);
    }

    // Main loop
    //int startTime = Time::now().toSec(), currentTime = startTime;
    while(ros::ok())
    {
        drive(pub);

        spinOnce();
        rate.sleep();
    }
    pthread_exit(NULL);
    return 0;
}