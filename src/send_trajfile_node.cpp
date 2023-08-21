#include "lemniscate_demo/send_trajfile_node.h"

using namespace std;

SetTrajectoryFile::SetTrajectoryFile(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    initPublishers();
}

void SetTrajectoryFile::initPublishers(){
    filepath_pub = nh_.advertise<std_msgs::String>("csv_file_path", 1);
}

void SetTrajectoryFile::wait_for_csv_path(){
    ROS_INFO("Input trajecotry CSV filepath:");
    string fp;
    cin >> fp;
    std_msgs::String fpt;
    fpt.data = fp;
    filepath_pub.publish(fpt);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_trajfile_node");
    ros::NodeHandle nh;
    SetTrajectoryFile cmder(&nh);
    while(ros::ok()){
        cmder.wait_for_csv_path();
        ros::spinOnce();
    }
    return 0;
}