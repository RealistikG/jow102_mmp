// Include ros lib
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <fstream>

//char name = "test";
double xPos = 0, yPos = 0;

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
    logFile.open ("/impacs/jow102/catkin_ws/src/jow102_mmp/poseLog.csv");

    // Loop rate
    ros::Rate rate(10);
    // Initial startup
    //ros::spinOnce();
    //rate.sleep();

    // Main loop
    while(ros::ok())
    {
        ros::spinOnce();

        logFile << xPos << "," << yPos << std::endl; //%s, %d", string1, string2, double1 << endl;

        // Short loop for testing
        double startTime = ros::Time::now().toSec(), currentTime = startTime;
        while(currentTime-startTime<0.5){
            currentTime = ros::Time::now().toSec();
            rate.sleep();
        }
    }

    logFile.close();
    return 0;
}