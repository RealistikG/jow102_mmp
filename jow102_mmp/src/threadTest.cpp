#include <iostream>
#include <cstdlib>
#include <pthread.h>
#include <ros/ros.h>


using namespace std;
int counter=0;


void *processThread(void *paramID) {
   long tid;
   tid = (long)paramID;
   cout << "Hello World! Thread ID, " << tid << endl;
   ros::Rate rate(1);
   while(ros::ok()) 
   {
      ROS_INFO_STREAM("Thread reading shared var:"
      << " counter=" << counter);
      rate.sleep();
   }
   pthread_exit(NULL);
}

int main (int argc, char **argv) {
   pthread_t thread;
   int rc;

   ros::init(argc, argv, "publish_velocity");
   ros::NodeHandle nh;


   cout << "main() : creating thread, "<< endl;
   rc = pthread_create(&thread, NULL, processThread, (void *)1);
      
   if (rc) {
      cout << "Error:unable to create thread," << rc << endl;
      exit(-1);
   }

   ros::Rate rate(5);
   while(ros::ok()) {
      counter++;
      rate.sleep();
   }
   
   
   pthread_exit(NULL);
}