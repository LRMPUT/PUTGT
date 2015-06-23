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
	std::ofstream outFile("groundtruth.txt");

	std::string tmp;
	std::getline(cameraZnacznik, tmp);
	std::getline(cameraZnacznik, tmp);

	float x,y,z,qw,qx,qy,qz;
	cameraZnacznik>>x>>y>>z>>qw>>qx>>qy>>qz;

	std::cout<<x <<" " << y<< " " << z<< " " << qw<< " "<<qx<<" "<<qy<< " " << qz<<std::endl;
	//	-0.11474,-0.18851,-0.078635,0.5129,-0.48145,-0.49094,0.51393
	// float qw = 0.5129f, qx = 0.48145, qy = -0.49094, qz = 0.51393;
//	x = -0.11474;
//	y = -0.18851;
//	z = -0.078635;
//	qw = 0.5129f, qx = -0.48145, qy = -0.49094, qz = 0.51393;

	Eigen::Quaternionf quat(qw, qx, qy, qz);

	Eigen::Matrix4f znacznik;
	znacznik.block<3, 3>(0, 0) = quat.toRotationMatrix();
	znacznik(0, 3) = x;
	znacznik(1, 3) = y;
	znacznik(2, 3) = z;
	znacznik(3, 3) = 1.0;

	int i = 0;
	float dist = 0.0f;
	float prevX = 0.0f, prevY = 0.0f, prevZ = 0.0f;
	while (!inFile.eof()) {
		std::string id, x, y, z, rod1, rod2, rod3;
		getline(inFile, id, ';');
		getline(inFile, rod1, ';');
		getline(inFile, rod2, ';');
		getline(inFile, rod3, ';');
		getline(inFile, x, ';');
		getline(inFile, y, ';');
		getline(inFile, z);

		if (id.size() < 1)
			break;

	//	std::cout << "IN: " << x << " " << y << " " << z << " " << rod1 << " "
	//			<< rod2 << " " << rod3 << std::endl;

		// Rodrigues to rotation matrix
		cv::Mat src(1, 3, CV_32FC1), dst(3, 3, CV_32FC1);
		src.at<float>(0, 0) = atof(rod1.c_str());
		src.at<float>(0, 1) = atof(rod2.c_str());
		src.at<float>(0, 2) = atof(rod3.c_str());
		cv::Rodrigues(src, dst);

		// Lets go to Eigen
		Eigen::Matrix3f matrix;
		cv::cv2eigen(dst, matrix);

		// Filling up translation information
		Eigen::Matrix4f mat;
		mat.block<3, 3>(0, 0) = matrix;
		mat(0, 3) = atof(x.c_str());
		mat(1, 3) = atof(y.c_str());
		mat(2, 3) = atof(z.c_str());
		mat(3, 3) = 1.0;

		// Compute resulting position in Kinect coordinate system
		Eigen::Matrix4f resultingPosition = mat * znacznik;

		Eigen::Quaternionf quat(resultingPosition.block<3, 3>(0, 0));
		outFile << id << " " << resultingPosition(0, 3) << " "
				<< resultingPosition(1, 3) << " " << resultingPosition(2, 3)
				<< " " << quat.x() << " " << quat.y() << " " << quat.z() << " "
				<< quat.w() << std::endl;

		// Speed
		float actX = resultingPosition(0, 3), actY = resultingPosition(1, 3),
				actZ = resultingPosition(2, 3);
		if (i == 0) {
			prevX = actX;
			prevY = actY;
			prevZ = actZ;
		}

		dist = dist
				+ sqrt(
						pow(actX - prevX, 2) + pow(actY - prevY, 2) + pow(actZ - prevZ, 2)
								);

		prevX = actX;
		prevY = actY;
		prevZ = actZ;
		i++;
	}

	std::cout << "Distance = " << dist << " m" << std::endl;
	std::cout << "Speed = " << 11 * dist / i << " m/s" << std::endl;

	inFile.close();
	cameraZnacznik.close();
	outFile.close();
}
