#ifndef COMMANDER_H
#define COMMANDER_H

#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace Eigen;

class TrajCommander
{
private:
    ros::NodeHandle nh_;
    ros::Time t_now;
    ros::Time t_traj_start;
    VectorXd traj_ts;
    MatrixXd traj_refs;
    int waypt_num;
    bool traj_loaded;
    bool allow_velctl, allow_accctl;

    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    ros::Subscriber local_pos_sub;
    ros::Publisher local_vel_pub;
    ros::Subscriber local_vel_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    ros::Subscriber csvfile_sub;
    ros::Subscriber cmdmode_sub;

    void initSubscribers();
    void initPublishers();
    void initServices();
    void initTimers();
    void subStateCb(const mavros_msgs::State::ConstPtr& msg);
    void subLocalPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void subLocalVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void subCsvfileCb(const std_msgs::String::ConstPtr& msg);
    void subCmdModeCb(const std_msgs::Int8::ConstPtr& msg);

public:
    ros::Timer timer1;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_pos;
    geometry_msgs::TwistStamped current_vel;
    bool takeoff;
    int cmd_mode;  // mode 0: position + velocity; mode 1: position
    Vector3d start_hover_pos;
    int at_pos_count;

    TrajCommander(ros::NodeHandle* nodehandle);
    ~TrajCommander(){};

    bool loadTrajCsv(std::string filepath);
    Vector3d calcRefPos(ros::Duration dt_now);
    Vector3d calcRefVel(ros::Duration dt_now);
    void timer1Cb(const ros::TimerEvent&);
};


#endif
