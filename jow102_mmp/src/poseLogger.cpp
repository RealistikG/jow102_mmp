// Include ros lib
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <fstream>

//char name = "test";
int xPos = 0, yPos = 0;

void logger_cb(const gazebo_msgs::ModelStates& msg)
{
    //name = msg.name;
    xPos = msg.pose[0].position.x;
    yPos = msg.pose[0].position.y;
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "Pose_Logger");
    ros::NodeHandle mainNh;

    ros::Subscriber sub = mainNh.subscribe("gazebo/model_states", 10, &logger_cb);

    std::ofstream logFile;
    logFile.open ("poseLog.csv");

    // Loop rate
    ros::Rate rate(10);
    // Initial startup
    ros::spinOnce();
    rate.sleep();

    // Main loop
    while(ros::ok())
    {
        ros::spinOnce();
        logFile << "%d, %d, \n", xPos, yPos; //%s, %d", string1, string2, double1 << endl;
        rate.sleep();
    }
    return 0;
}