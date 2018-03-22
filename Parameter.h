#pragma once
#ifndef PARASPRE
#define PARASPRE
#include <fstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <unordered_map>
#include "Eigen/Eigen"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
using namespace std;
class Signal
{
public:
	Signal() {
		color = "white";
		MapPointId = -1;
		startFrame = -1;
		endFrame = -1;
		trackStatus = "untracked";
		worldPosition = Eigen::Vector3f::Zero();
		r = g = b = a = 255;
	}
public:
	string name;
	string color;
	int MapPointId;
	int startFrame;
	int endFrame;
	string trackStatus;
	Eigen::Vector3f worldPosition;
	unsigned char r, g, b, a;

};
struct Point
{
	Eigen::Vector3f worldPosition;
	unsigned char r, g, b, a;
};

struct Label
{
	size_t frameIndex;
	cv::Rect position;
	string color;
};
class Cam_intrinsic
{
   public:
	float fx, fy, cx, cy;
	Eigen::Matrix3f INTRINSIC;
	float distortion[5];
	void fill_INTRINSIC()
	{
		INTRINSIC = Eigen::Matrix3f::Identity();
		INTRINSIC(0, 0) = fx;
		INTRINSIC(1, 1) = fy;
		INTRINSIC(0, 2) = cx;
		INTRINSIC(1, 2) = cy;
	}
};
class ParameterReader
{
public:
	//void readLine(ifstream &fin, float &value);
	//void readSignal(ifstream &fin, string sname);
	//void readCamIntrinsic(ifstream &fin, string fx_value);
	//string getData(string key);
	//void readLabel(string path);
	//vector<Label> getLabels(int frameIndex);
	vector<Label> getLabels(int frameIndex)
	{
		std::vector<Label> tmp;
		auto iter = LabelData.find(frameIndex);
		if (iter == LabelData.end())
			return tmp;
		else
			return iter->second;
	}
	void readLine(ifstream &fin, float &value)
	{
		string LineStr;
		getline(fin, LineStr);
		int pos = LineStr.find("=");
		if (pos == -1)
			return;
		string key = LineStr.substr(0, pos);
		value = stof(LineStr.substr(pos + 1, LineStr.length()));
	}
	void readCamIntrinsic(ifstream &fin, string fx_value)
	{
		cam_intrinsic.fx = stof(fx_value);
		string fyLine, cxLine, cyLine, distLine;
		string key, value;
		readLine(fin, cam_intrinsic.fy);
		readLine(fin, cam_intrinsic.cx);
		readLine(fin, cam_intrinsic.cy);
		string distortLine;
		getline(fin, distortLine);
		size_t pos = distortLine.find("=");
		string distor = distortLine.substr(pos + 1, distortLine.length());
		stringstream ss_distor;
		ss_distor.str(distor);
		string d0, d1, d2, d3, d4;
		ss_distor >> d0 >> d1 >> d2 >> d3 >> d4;
		cam_intrinsic.distortion[0] = stof(d0);
		cam_intrinsic.distortion[1] = stof(d1);
		cam_intrinsic.distortion[2] = stof(d2);
		cam_intrinsic.distortion[3] = stof(d3);
		cam_intrinsic.distortion[4] = stof(d4);
		cam_intrinsic.fill_INTRINSIC();
		printf("Get camera intrinsic !\n");
	}

	string getData(string key)
	{
		unordered_map <string, string>::iterator iter = data.find(key);
		if (iter == data.end())
		{
			cerr << "Parameter name " << key << "not found!" << endl;
			return string("NOT FOUND!");
		}
		return iter->second; //second is the value, first is the key
	}
	void readLabel(string path)
	{
		//cout<<"/home/sensetime/Downloads/GOD/label_157_part.txt"<<endl
		ifstream iter(path);
		if (!iter.is_open()){
			cerr<<"failed!\n";
		}
		int start = stoi(getData("rec_start"));
		int end = stoi(getData("rec_end"));
		int LastFid = 0;
		std::vector< Label> Labels1Frame;
		while (!iter.eof())
		{
			string str;
			getline(iter, str);
			int length = str.length();
			if (length < 12)
				continue;
			size_t posit = 4;
			int fId = stoi(str.substr(posit, 5));
			if (fId < start || fId > end)
				continue;
			stringstream ss;
			ss.str(str);
			string name,type, probability, px, py, qx, qy;
			ss >> name >>type >>probability >> px >> py >> qx >> qy;
			if (fId != LastFid && LastFid != 0)
			{
				LabelData[LastFid] = Labels1Frame;
				Labels1Frame.clear();
			}
			Label label;
			label.frameIndex = fId;
			label.position = cv::Rect(stoi(px), stoi(py), stoi(qx)-stoi(px), stoi(qy)-stoi(py));
			label.color = type;
			Labels1Frame.push_back(label);
			LastFid = fId;
		}
	}
	void readTracjectory(string tPath)
	{
		ifstream fin(tPath);
		if(!fin.is_open()){
			cerr<<"failed to load tracjectory!\n";
			return;
		} 
		float tm;
		float linedata[12];
        Eigen::Matrix4f Pose;
		while (!fin.eof())
		{
			string line;
            getline(fin,line);
			if(sscanf(line.c_str(), "%f %f %f %f %f %f %f %f", &tm, &linedata[0],&linedata[1],&linedata[2],
                                    &linedata[3],&linedata[4],&linedata[5],&linedata[6]) == 8)
                                    //q0,     q1,      q2 ,     q3
			{
				//quaternion ->R
				Pose(0,0) = 1 - 2*linedata[5]*linedata[5] - 2*linedata[6]*linedata[6];
				Pose(0,1) = 2*linedata[4]*linedata[5] + 2*linedata[3]*linedata[6];
				Pose(0,2) = 2*linedata[4]*linedata[6] - 2*linedata[3]*linedata[5];
				Pose(1,0) = 2*linedata[4]*linedata[5] - 2*linedata[3]*linedata[6];
				Pose(1,1) = 1 - 2*linedata[4]*linedata[4] - 2*linedata[6]*linedata[6];
				Pose(1,2) = 2*linedata[5]*linedata[6] + 2*linedata[3]*linedata[4];
				Pose(2,0) = 2*linedata[4]*linedata[6] + 2*linedata[3]*linedata[5];
				Pose(2,1) = 2*linedata[5]*linedata[6] - 2*linedata[3]*linedata[4];
				Pose(2,2) = 1 - 2*linedata[4]*linedata[4] - 2*linedata[5]*linedata[5];
				//T
				Pose(0,3) = linedata[0];
				Pose(1,3) = linedata[1];
				Pose(2,3) = linedata[2];
				// 0 0 0 1
				Pose(3,0) = Pose(3,1) = Pose(3,2) = 0;
				Pose(3,3) = 1;
				// center = -Pose.block<3,3>(0,0).transpose()*Pose.block<3,1>(0,3);
				// center = Eigen::Vector3d(linedata[0],linedata[1],linedata[2]);
				trajectory.push_back(Pose);
			}
		}
	}
	ParameterReader(string pFile)
	{
		ifstream fin(pFile.c_str());
		if (!fin)
		{
			cerr << "parameter file missed" << endl;
			return;
		}
		while (!fin.eof())
		{
			string str;
			getline(fin, str);
			if (str[0] == '#')
			{
				continue;
			}
			int pos = str.find("=");
			if (pos == -1)
				continue;
			string key = str.substr(0, pos);
			string value = str.substr(pos + 1, str.length()-pos-1);
			if(value[value.size()-1] == 13)//去除换行
			{
				value.pop_back();
			}
			data[key] = value;
			if (key == "fx")
			{
				readCamIntrinsic(fin, value);
			}
			//include all bad situations
			if (!fin.good())
				break;
		}
		readLabel(getData("label_path"));
		cout<<"collected "<<data.size()<<" paras!\n";
		readTracjectory(getData("trajectroy_path"));
		printf("loaded tracjectory total %zd frame \n",trajectory.size());
	}
public:
	unordered_map <string, string> data;//parameter.txt ÆäËû²ÎÊý
	vector<Eigen::Matrix4f> trajectory;//result
	unordered_map<int,vector< Label>> LabelData;//label.txt
	Cam_intrinsic cam_intrinsic;
};


#endif // PARASPRE