// include ros lib
#include <ros/ros.h>

// include opencv2 lib
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";
cv_bridge::CvImagePtr cv_ptr;


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

    // Pixel coordinates on image from which it will be warped
    float w = 104, h = 233;
    Point2f src_l[4] = {{246, 252},{268, 265},{16, 292},{105, 346}};
    Point2f src_r[4] = {{391,265},{427,259},{540,355},{622,300}};
    Point2f dst[4] = {{0.0f, 0.0f},{w, 0.0f},{0.0f, h},{w, h}};

    // Warp image so left adn right lines can be tracked more easily
    Mat matrix_l, matrix_r, imgWarp_l, imgWarp_r;
    matrix_l = getPerspectiveTransform(src_l, dst);
    matrix_r = getPerspectiveTransform(src_r, dst);
    warpPerspective(cv_ptr->image, imgWarp_l, matrix_l, Point(w,h));
    warpPerspective(cv_ptr->image, imgWarp_r, matrix_r, Point(w,h));

    // Update GUI Window
    imshow(OPENCV_WINDOW, cv_ptr->image);
    imshow("img_l", imgWarp_l);
    imshow("img_r", imgWarp_r);

    waitKey(3);

    //imwrite("/impacs/jow102/catkin_ws/src/jow102_mmp/test_img.jpg", cv_ptr->image); //save test image
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
        //break
    }
    return 0;
}



