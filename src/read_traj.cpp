#include <fstream>
#include <sstream>
#include "lemniscate_demo/traj_commander.h"

using namespace std;

template<typename T>
T StringToNum(const string &s)
{
	istringstream iss(s);
	T num;
	iss>>num;
	return num;
}

void spilt(const string &line, const string &c, vector<string> &v)
{
	int pos1 = 0;
	int pos2 = line.find(c);
	while(string::npos != pos2)
	{
		v.push_back(line.substr(pos1,pos2-pos1));
		pos1 += pos2+c.size();
		pos2 = line.find(c,pos1);
	}
}

bool TrajCommander::loadTrajCsv(string filepath){
    ifstream inFile(filepath, ios::in);
    if(inFile.fail())
	{
		ROS_WARN("File not found!");
        traj_loaded = false;
		return false;
	}
	std::string lineStr;
	vector<vector<string>> strArray;
	while (getline(inFile, lineStr) && inFile.good())
	{
		// cout << lineStr << endl;
		stringstream ss(lineStr);
		string str;
		vector<string> lineArray;
		while (getline(ss, str, ','))
			lineArray.push_back(str);
		strArray.push_back(lineArray);
	}
    int ts_idx = -1, \
        px_idx = -1, py_idx = -1, pz_idx = -1, \
        vx_idx = -1, vy_idx = -1, vz_idx = -1, \
        ax_idx = -1, ay_idx = -1, az_idx = -1;
    for(int i=0; i<strArray[0].size(); i++){
        if(strArray[0][i] == "ts"){
            ts_idx = i;
        }
        if(strArray[0][i] == "px"){
            px_idx = i;
        }
        if(strArray[0][i] == "py"){
            py_idx = i;
        }
        if(strArray[0][i] == "pz"){
            pz_idx = i;
        }
        if(strArray[0][i] == "vx"){
            vx_idx = i;
        }
        if(strArray[0][i] == "vy"){
            vy_idx = i;
        }
        if(strArray[0][i] == "vz"){
            vz_idx = i;
        }
        if(strArray[0][i] == "ax"){
            ax_idx = i;
        }
        if(strArray[0][i] == "ay"){
            ay_idx = i;
        }
        if(strArray[0][i] == "az"){
            az_idx = i;
        }
    }
    if(ts_idx < 0 || px_idx < 0 || py_idx < 0 || pz_idx < 0){
        ROS_INFO("File not found!");
        traj_loaded = false;
		return false;
    }
    if(vx_idx < 0 || vy_idx < 0 || vz_idx < 0){
        allow_velctl = false;
    }
    else{
        allow_velctl = true;
    }
    if(ax_idx < 0 || ay_idx < 0 || az_idx < 0){
        allow_accctl = false;
    }
    else{
        allow_accctl = true;
    }
    waypt_num = strArray.size() - 1;
    if(waypt_num <= 0){
        return false;
    }
    traj_ts = VectorXd(waypt_num);
    traj_refs = MatrixXd(waypt_num, 6);
    for(int i=0; i<waypt_num; i++){
        traj_ts(i) = StringToNum<double>(strArray[i+1][ts_idx]);
        traj_refs(i,0) = StringToNum<double>(strArray[i+1][px_idx]);
        traj_refs(i,1) = StringToNum<double>(strArray[i+1][py_idx]);
        traj_refs(i,2) = StringToNum<double>(strArray[i+1][pz_idx]);
        if(allow_velctl){
            traj_refs(i,3) = StringToNum<double>(strArray[i+1][vx_idx]);
            traj_refs(i,4) = StringToNum<double>(strArray[i+1][vy_idx]);
            traj_refs(i,5) = StringToNum<double>(strArray[i+1][vz_idx]);
        }
    }
    traj_loaded = true;
    start_hover_pos = traj_refs.topLeftCorner(1,3).transpose();
    return true;
}