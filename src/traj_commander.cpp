#include "lemniscate_demo/traj_commander.h"

#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>

TrajCommander::TrajCommander(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    initSubscribers();
    initPublishers();
    initServices();
    initTimers();
    takeoff = false;
    traj_loaded = false;
    start_hover_pos = Vector3d(0.0, 0.0, 1.5);
    at_pos_count = 0;
    cmd_mode = 0;
    allow_velctl = false;
    waypt_num = 0;
}

void TrajCommander::initSubscribers(){
    state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &TrajCommander::subStateCb, this);
    local_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &TrajCommander::subLocalPosCb, this);
    local_vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 10, &TrajCommander::subLocalVelCb, this);
    csvfile_sub = nh_.subscribe<std_msgs::String>("csv_file_path", 1, &TrajCommander::subCsvfileCb, this);
    cmdmode_sub = nh_.subscribe<std_msgs::Int8>("control_mode_cmd", 1, &TrajCommander::subCmdModeCb, this);
}

void TrajCommander::initPublishers(){
    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    local_vel_pub = nh_.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
}

void TrajCommander::initServices(){
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

void TrajCommander::initTimers(){
    timer1 = nh_.createTimer(ros::Rate(20), &TrajCommander::timer1Cb, this);
}

void TrajCommander::subStateCb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void TrajCommander::subLocalPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = *msg;
}

void TrajCommander::subLocalVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_vel = *msg;
}

void TrajCommander::subCsvfileCb(const std_msgs::String::ConstPtr& msg){
    traj_loaded = false;
    bool flag = loadTrajCsv(msg->data);
    if(flag){
        ROS_INFO("Trajectory at %s loaded!", msg->data);
        takeoff = false;
    }
    else{
        ROS_INFO("Invalid trajectory file path.");
    }
}

void TrajCommander::subCmdModeCb(const std_msgs::Int8::ConstPtr& msg){
    int m = msg->data;
    if(m != 0 && m != 1){
        ROS_INFO("Control mode not supported!");
    }
    else{
        cmd_mode = m;
    }
}


Vector3d TrajCommander::calcRefPos(ros::Duration dt_now){
    if(!traj_loaded){
        return start_hover_pos;
    }
    double _t = dt_now.toSec();
    if(_t < 0){
        ROS_INFO("Invalid time stamp!");
        return start_hover_pos;
    }
    if(_t >= traj_ts(waypt_num-1)){
        return traj_refs.bottomLeftCorner(1,3).transpose();
    }
    for(int i=0; i<waypt_num-1; i++){
        if(_t >= traj_ts(i) && _t < traj_ts(i+1)){
            double frac = (_t-traj_ts(i)) / (traj_ts(i+1)-traj_ts(i));
            Vector3d _start = traj_refs.row(i).block<1,3>(0,0).transpose();
            Vector3d _end = traj_refs.row(i+1).block<1,3>(0,0).transpose();
            return _start + frac*(_end - _start);  // Linear interpolation
        }
    }
    ROS_INFO("Impossible situation.");
    return start_hover_pos;
}

Vector3d TrajCommander::calcRefVel(ros::Duration dt_now){
    if(!traj_loaded || !allow_velctl){
        return Vector3d::Zero();
    }
    double _t = dt_now.toSec();
    if(_t < 0){
        ROS_INFO("Invalid time stamp!");
        return Vector3d::Zero();
    }
    if(_t >= traj_ts(waypt_num-1)){
        return traj_refs.bottomRows(1).block<1,3>(0,3).transpose();
    }
    for(int i=0; i<waypt_num-1; i++){
        if(_t >= traj_ts(i) && _t < traj_ts(i+1)){
            double frac = (_t-traj_ts(i)) / (traj_ts(i+1)-traj_ts(i));
            Vector3d _start = traj_refs.row(i).block<1,3>(0,3).transpose();
            Vector3d _end = traj_refs.row(i+1).block<1,3>(0,3).transpose();
            return _start + frac*(_end - _start);  // Linear interpolation
        }
    }
    ROS_INFO("Impossible situation.");
    return Vector3d::Zero();
}

void TrajCommander::timer1Cb(const ros::TimerEvent&){
    t_now = ros::Time::now();
    if(current_state.connected){
        geometry_msgs::PoseStamped pos_cmd;
        if(current_state.mode=="OFFBOARD" && current_state.armed){
            if(!traj_loaded){
                pos_cmd.pose.position.x = start_hover_pos(0);
                pos_cmd.pose.position.y = start_hover_pos(1);
                pos_cmd.pose.position.z = start_hover_pos(2);
                t_traj_start = t_now;
            }
            else if(!takeoff){
                pos_cmd.pose.position.x = start_hover_pos(0);
                pos_cmd.pose.position.y = start_hover_pos(1);
                pos_cmd.pose.position.z = start_hover_pos(2);
                if(pow(current_pos.pose.position.x-pos_cmd.pose.position.x,2) +
                    pow(current_pos.pose.position.y-pos_cmd.pose.position.y,2) +
                    pow(current_pos.pose.position.z-pos_cmd.pose.position.z,2) <= 0.01){
                    if(at_pos_count > 150){
                        at_pos_count = 0;
                        takeoff = true;
                        ROS_INFO_ONCE("Start trajectory command!");
                    }
                    else{
                        at_pos_count++;
                    }
                }
                else{
                    at_pos_count = 0;
                }
                t_traj_start = t_now;
            }
            else{
                Vector3d cmd_pos = calcRefPos(t_now-t_traj_start);
                pos_cmd.pose.position.x = cmd_pos(0);
                pos_cmd.pose.position.y = cmd_pos(1);
                pos_cmd.pose.position.z = cmd_pos(2);
                if(allow_velctl && cmd_mode==0){
                    Vector3d cmd_vel = calcRefVel(t_now-t_traj_start);
                    geometry_msgs::Twist vel_cmd;
                    vel_cmd.linear.x = cmd_vel(0);
                    vel_cmd.linear.y = cmd_vel(1);
                    vel_cmd.linear.z = cmd_vel(2);
                    local_vel_pub.publish(vel_cmd);
                    ROS_INFO("Commanded position is: %f %f %f; velocity is: %f %f %f", \
                                cmd_pos(0), cmd_pos(1), cmd_pos(2), cmd_vel(0), cmd_vel(1), cmd_vel(2));
                }
                else{
                    ROS_INFO("Commanded position is: %f %f %f", cmd_pos(0), cmd_pos(1), cmd_pos(2));
                }
                start_hover_pos = cmd_pos;
            }
        }
        else{
            pos_cmd.pose.position.x = start_hover_pos(0);
            pos_cmd.pose.position.y = start_hover_pos(1);
            pos_cmd.pose.position.z = start_hover_pos(2);
            ROS_INFO_ONCE("Sending meaningless target...");
        }
        local_pos_pub.publish(pos_cmd);
    }
    else{
        ROS_INFO("Unconnected!");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    TrajCommander cmder(&nh);

    if(argc >= 2){
        std::string _file = argv[1];
        cmder.loadTrajCsv(_file);
    }

    ros::spin();
    return 0;
}