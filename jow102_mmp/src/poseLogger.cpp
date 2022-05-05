// Include ros lib
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <fstream>

string name = "test";

void logger_cb(const gazebo_msgs::ModelStates& msg)
{
    name = msg.name;
}

int main(int argc, char **argv) {
    // Initialize ROS
    init(argc, argv, "Pose_Logger");
    NodeHandle mainNh;

    ros::Subscriber sub = mainNh.subscribe("gazebo/model_states", 10, &logger_cb);

    std::ofstream logFile;
    logFile.open ("poseLog.csv");

    // Loop rate
    ros::Rate rate(10);
    // Initial startup
    spinOnce();
    rate.sleep();

    // Main loop
    while(ros::ok())
    {
        spinOnce();
        logFile << "%s", name, std::endl; //%s, %d", string1, string2, double1 << endl;
        rate.sleep();
    }
    return 0;
}