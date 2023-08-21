#ifndef SET_CSVPATH_NODE
#define SET_CSVPATH_NODE

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>

class SetTrajectoryFile
{
private:
    ros::NodeHandle nh_;
    ros::Publisher filepath_pub;
    void initPublishers();

public:
    SetTrajectoryFile(ros::NodeHandle* nodehandle);
    ~SetTrajectoryFile(){};
    void wait_for_csv_path();
};


#endif