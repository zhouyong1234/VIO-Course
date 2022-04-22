#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/dataset/EuRoC/MH-05/mav0/";
string sConfig_path = "../config/";

std::shared_ptr<System> pSystem;


void PubImuData()
{
	// string sImu_data_file = sConfig_path + "MH_05_imu0.txt";
    string sImu_data_file = sData_path + "imu_pose_noise.txt";
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
    double tmp;
	Vector3d vAcc;
	Vector3d vGyr;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec;
        for(int i = 0; i < 7; i++)
            ssImuData >> tmp;
        ssImuData >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec, vGyr, vAcc);
		usleep(5000*nDelayTimes);
	}
	fsImu.close();
}


void PubImageData()
{
    string sImage_file = sData_path + "cam_pose.txt"; // 含时间戳的文件
	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}
	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;
    int n = 0;
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImgData(sImage_line);
		ssImgData >> dStampNSec;
        string all_points_file_name = "/home/touchair/vio_data_simulation/bin/keyframe/all_points_" + to_string(n)+ ".txt";  //第n个相机对应的观测数据的文件名
        vector<cv::Point2f> FeaturePoints;
        std::ifstream f;
        f.open(all_points_file_name);
        while(!f.eof())
        {
            std::string s;
            std::getline(f, s);
            if(!s.empty())
            {
                std::stringstream ss;
                ss << s;
                double tmp;
                for(int i = 0; i < 4; i++)
                    ss >> tmp;
                float px, py;
                ss >> px;
                ss >> py;
                cv::Point2f pt(px, py);
                FeaturePoints.push_back(pt);
            }
        }

		pSystem->PubSimImageData(dStampNSec, FeaturePoints);
		usleep(50000*nDelayTimes);
        n++;
	}
	fsImage.close();
}


int main(int argc, char **argv)
{
	if(argc != 3)
	{
		cerr << "./simulation-test 特征点文件路径 配置文件/config \n" 
			 << endl;
		return -1;
	}
	sData_path = argv[1];
	sConfig_path = argv[2];

	pSystem.reset(new System(sConfig_path));
	
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem); // 最重要！！
		
	// sleep(5);
	std::thread thd_PubImuData(PubImuData); // 获取IMU数据的线程

	std::thread thd_PubImageData(PubImageData); //获取图像数据的线程
	
	std::thread thd_Draw(&System::Draw, pSystem); // 画图的线程


	thd_PubImuData.join();
	thd_PubImageData.join();
    thd_BackEnd.join();
    
    thd_Draw.join();

	cout << "main end... see you ..." << endl;
	return 0;
}