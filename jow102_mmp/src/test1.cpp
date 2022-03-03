// include ros lib
#include <ros/ros.h>

// include opencv2 lib
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "Image window";
cv_bridge::CvImagePtr cv_ptr;
using namespace cv;

void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Draw circle of radius 10 at coordinates (0, 0)
    //cv::circle(cv_ptr->image, cv::Point(0, 0), 10, CV_RGB(255,0,0));

    float w = 45, h = 221;
    Point2f src[4] = {{233, 262},{248, 268},{19, 316},{58, 339}};
    Point2f dst[4] = {{0.0f, 0.0f},{w, 0.0f},{0.0f, h},{w, h}};

    Mat matrix, imgWarp;
    matrix = getPerspectiveTransform(src, dst);
    warpPerspective(cv_ptr->image, imgWarp, matrix, Point(w,h));

    // Update GUI Window
    imshow(OPENCV_WINDOW, cv_ptr->image);
    imshow("Warped img", imgWarp);
    waitKey(3);

    //cv::imwrite("/impacs/jow102/catkin_ws/src/jow102_mmp/test_img.jpg", cv_ptr->image); //save test image
}

int main(int argc, char **argv) {
    // Initialize the ROS system.
    ros::init(argc, argv, "test");

    // Establish this program as a ROS node.
    ros::NodeHandle nh;

    // Subscribe to input image topic using image transport.
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, image_cb);
    image_transport::Publisher pub = it.advertise("/image_converter/output_video", 1);
    namedWindow(OPENCV_WINDOW);

    //ros::spinOnce();
    while(true){
        ros::spinOnce();
        pub.publish(cv_ptr->toImageMsg());
    }
    return 0;
}



