// Include ros lib
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <fstream>

// X&Y positions
double xPos = 0, yPos = 0;

void logger_cb(const gazebo_msgs::ModelStates& msg)
{
    // Set X&Y positions
    xPos = msg.pose[1].position.x;
    yPos = msg.pose[1].position.y;
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "Pose_Logger");
    ros::NodeHandle mainNh;
    // Subscriber to gazebo/model_states
    ros::Subscriber sub = mainNh.subscribe("gazebo/model_states", 10, &logger_cb);
    // Open log file
    std::ofstream logFile;
    logFile.open ("/impacs/jow102/catkin_ws/src/jow102_mmp/poseLog.csv");
    // Loop rate
    ros::Rate rate(10);
    // Main loop
    while(ros::ok())
    {
        ros::spinOnce();
        // Write to log file
        logFile << xPos << "," << yPos << std::endl; //%s, %d", string1, string2, double1 << endl;
        // Timed loop using ros time - only allows 2 logs per second
        double startTime = ros::Time::now().toSec(), currentTime = startTime;
        while(currentTime-startTime<0.5){
            currentTime = ros::Time::now().toSec();
            rate.sleep();
        }
    }
    logFile.close();
    return 0;
}