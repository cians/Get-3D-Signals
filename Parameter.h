#pragma once
#ifndef _PARAMETER_H_
#define _PARAMETER_H_
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

using std::string;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;

struct Label
{
	int SID;
	size_t frameIndex;
	cv::Rect position;
	string color;
};
class Signal
{
public:
	Signal() {
		color = "white";
		SID= -1;
		startFrame = -1;
		endFrame = -1;
		trackStatus = "untracked";
		LeftUpPos = Eigen::Vector3f::Zero();
		r = g = b = a = 255;
	}
public:
	string name;
	string color;
	int SID;
	int startFrame;
	int endFrame;
	string trackStatus;
	Eigen::Vector3f LeftUpPos;
	Eigen::Vector3f RightDownPos;
	Eigen::Vector2i speed;
	cv::Rect CurFrameRect;
	unsigned char r, g, b, a;
	std::vector<Label> ObsLabels;

};

class CamParams
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
	//void readLine(std::ifstream &fin, float &value);
	//void readSignal(std::ifstream &fin, string sname);
	//void readCamIntrinsic(std::ifstream &fin, string fx_value);
	//string getData(string key);
	//void readLabel(string path);
	//vector<Label> getLabels(int frameIndex);
	std::vector<Label>* getLabels(int frameIndex)
	{
		auto iter = LabelData.find(frameIndex);
		if (iter == LabelData.end())
			return nullptr;
		else
			return &(iter->second);
	}
	void readLine(std::ifstream &fin, float &value)
	{
		string LineStr;
		getline(fin, LineStr);
		int pos = LineStr.find("=");
		if (pos == -1)
			return;
		string key = LineStr.substr(0, pos);
		value = stof(LineStr.substr(pos + 1, LineStr.length()));
	}
	void readCamIntrinsic(std::ifstream &fin, string fx_value)
	{
		camParams.fx = stof(fx_value);
		string fyLine, cxLine, cyLine, distLine;
		string key, value;
		readLine(fin, camParams.fy);
		readLine(fin, camParams.cx);
		readLine(fin, camParams.cy);
		string distortLine;
		getline(fin, distortLine);
		size_t pos = distortLine.find("=");
		string distor = distortLine.substr(pos + 1, distortLine.length());
		std::stringstream ss_distor;
		ss_distor.str(distor);
		string d0, d1, d2, d3, d4;
		ss_distor >> d0 >> d1 >> d2 >> d3 >> d4;
		camParams.distortion[0] = stof(d0);
		camParams.distortion[1] = stof(d1);
		camParams.distortion[2] = stof(d2);
		camParams.distortion[3] = stof(d3);
		camParams.distortion[4] = stof(d4);
		camParams.fill_INTRINSIC();
		printf("Get camera intrinsic !\n");
	}

	string getData(string key)
	{
		std::unordered_map <string, string>::iterator iter = paramsMap.find(key);
		if (iter == paramsMap.end())
		{
			std::cerr << "Parameter name " << key << "not found!\n";
			return string("NOT FOUND!");
		}
		return iter->second; //second is the value, first is the key
	}
	void readLabel(string path)
	{
		//std::cout<<"/home/sensetime/Downloads/GOD/label_157_part.txt"<<endl
		std::ifstream fin(path);
		if (!fin.is_open()){
			std::cerr<<"failed!\n";
		}
		int startId = stoi(getData("rec_start"));
		int endId = stoi(getData("rec_end"));
		int LastFid = 0;
		std::vector< Label> Labels1Frame;
		while (!fin.eof())
		{
			string lineStr;
			getline(fin, lineStr);
			int length = lineStr.length();
			if (length < 12)
				continue;
			size_t posit = 4;                            //!Need change as your file format!!!
			int frameId = stoi(lineStr.substr(posit, 5));//!Need change as your file format!!!
			if (frameId < startId || frameId > endId)
				continue;
			std::stringstream lineStrStream;
			lineStrStream.str(lineStr);
			string name,type, probability, px, py, qx, qy;
			lineStrStream >> name >>type >>probability >> px >> py >> qx >> qy;
			if (frameId != LastFid && LastFid != 0)
			{
				LabelData.insert({LastFid,Labels1Frame});
				Labels1Frame.clear();
			}
			Label label;
			label.frameIndex = frameId;
			label.position = cv::Rect(stoi(px), stoi(py), stoi(qx)-stoi(px), stoi(qy)-stoi(py));
			label.color = type;
			Labels1Frame.push_back(label);
			LastFid = frameId;
		}
		printf("get %zd labelData!\n",LabelData.size());
	}
	void readTracjectory(string tPath)
	{
		std::ifstream fin(tPath);
		if(!fin.is_open()){
			std::cerr<<"failed to load tracjectory!\n";
			return;
		} 
		float tm;
		float linedata[12];
        Eigen::Matrix<float,3,4> Pose;
		while (!fin.eof())
		{
			string line;
            getline(fin,line);
	        if(sscanf(line.c_str(), "%f %f %f %f %f %f %f %f %f %f %f %f", &linedata[0],&linedata[1],&linedata[2],&linedata[3],&linedata[4],&linedata[5],
                                                               &linedata[6],&linedata[7],&linedata[8],&linedata[9],&linedata[10],&linedata[11]) == 12)
			{
				Pose<<linedata[0],linedata[1],linedata[2],linedata[3],
					linedata[4],linedata[5],linedata[6],linedata[7],
					linedata[8],linedata[9],linedata[10],linedata[11];
				trajectory.push_back(Pose);
			} else if(sscanf(line.c_str(), "%f %f %f %f %f %f %f %f", &tm, &linedata[0],&linedata[1],&linedata[2],
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
				// Pose(3,0) = Pose(3,1) = Pose(3,2) = 0;
				// Pose(3,3) = 1;
				// // center = -Pose.block<3,3>(0,0).transpose()*Pose.block<3,1>(0,3);
				// center = Eigen::Vector3d(linedata[0],linedata[1],linedata[2]);
				trajectory.push_back(Pose);
			}

		}
	}
	ParameterReader(){}
	ParameterReader(string pFile)
	{
		std::ifstream fin(pFile.c_str());
		if (!fin)
		{
			std::cerr << "parameter file missed\n";
			return;
		}
		while (!fin.eof())
		{
			string lineStr;
			getline(fin, lineStr);
			if (lineStr[0] == '#')
			{
				continue;
			}
			int pos = lineStr.find("=");
			if (pos == -1)
				continue;
			string key = lineStr.substr(0, pos);
			string value = lineStr.substr(pos + 1, lineStr.length()-pos-1);
			if(value[value.size()-1] == 13)//去除换行
			{
				value.pop_back();
			}
			paramsMap[key] = value;
			if (key == "fx")
			{
				readCamIntrinsic(fin, value);
			}
			//include all bad situations
			if (!fin.good())
				break;
		}
		readLabel(getData("label_path"));
		printf("collected %zd paras\n",paramsMap.size());
		readTracjectory(getData("trajectroy_path"));
		printf("loaded tracjectory total %zd frame \n",trajectory.size());
	}
public:
	std::unordered_map <string, string> paramsMap;//parameter.txt ÆäËû²ÎÊý
	std::vector<Eigen::Matrix<float,3,4> > trajectory;//result
	std::unordered_map<int,std::vector< Label>> LabelData;// may be need modify set to pointer;
	CamParams camParams;
};


#endif // _PARAMETER_H_