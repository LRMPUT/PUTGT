#include "opencv/cv.h"

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <fstream>
#include <string>

#include <math.h>

/*
 * argv[1] -> trajectory file
 *
 */
using namespace std;

int main(int argc, char *argv[]) {

	std::ifstream inFile(argv[1]), cameraZnacznik(argv[2]);
	std::string inFileName = std::string(argv[1]);
	std::string outFileName =  inFileName.substr(0, inFileName.find('.')) + "_recomputed.txt";
	std::ofstream outFile(outFileName);

	std::string tmp;
	std::getline(cameraZnacznik, tmp);
	std::getline(cameraZnacznik, tmp);

	double x,y,z,qw,qx,qy,qz;
	cameraZnacznik>>x>>y>>z>>qw>>qx>>qy>>qz;

	std::cout<<x <<" " << y<< " " << z<< " " << qw<< " "<<qx<<" "<<qy<< " " << qz<<std::endl;

	Eigen::Quaterniond quat(qw, qx, qy, qz);

	Eigen::Matrix4d znacznik;
	znacznik.block<3, 3>(0, 0) = quat.toRotationMatrix();
	znacznik(0, 3) = x;
	znacznik(1, 3) = y;
	znacznik(2, 3) = z;
	znacznik(3, 3) = 1.0;

	int i = 0;
	float dist = 0.0f;
	float prevX = 0.0f, prevY = 0.0f, prevZ = 0.0f;
	while (!inFile.eof()) {
		double id;
		inFile>>id>>x>>y>>z>>qx>>qy>>qz>>qw;

		Eigen::Quaterniond quat2(qw, qx, qy, qz);

		Eigen::Matrix4d kinectEstimate;
		kinectEstimate.block<3, 3>(0, 0) = quat2.toRotationMatrix();
		kinectEstimate(0, 3) = x;
		kinectEstimate(1, 3) = y;
		kinectEstimate(2, 3) = z;
		kinectEstimate(3, 3) = 1.0;


		// Compute resulting Kinect estimate in znacznik coordinate system
		Eigen::Matrix4d resultingPosition = kinectEstimate * znacznik.inverse();

		Eigen::Quaterniond quat(resultingPosition.block<3, 3>(0, 0));
		outFile << id << " " << resultingPosition(0, 3) << " "
				<< resultingPosition(1, 3) << " " << resultingPosition(2, 3)
				<< " " << quat.x() << " " << quat.y() << " " << quat.z() << " "
				<< quat.w() << std::endl;

		i++;
	}

	inFile.close();
	cameraZnacznik.close();
	outFile.close();
}
