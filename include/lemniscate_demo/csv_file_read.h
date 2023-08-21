#ifndef CSVREAD
#define CSVREAD

#include <iostream>
#include <vector>

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

#endif