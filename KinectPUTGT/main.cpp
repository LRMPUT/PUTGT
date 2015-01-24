#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>

// C++11: threads
#include <thread>

using namespace cv;
using namespace std;

#define CYCLIC_BUFFER_SIZE 5000

Mat *dMap;
Mat *rgbImg;
Mat *cameraImg;
int i = 0, j = 0;
bool experiment_end = 0;

// Saving depth data
//		k -> number of frame
void Dsave(int k) {
	char zapis[40];
	sprintf(zapis, "depth_%05d.png", k);

	imwrite(zapis, dMap[k % CYCLIC_BUFFER_SIZE]);
}

// Saving rgb data
//		k -> number of frame
void RGBsave(int k) {
	char zapis[40];
	sprintf(zapis, "rgb_%05d.bmp", k);
	imwrite(zapis, rgbImg[k % CYCLIC_BUFFER_SIZE]);
}

// Thread responsible for saving data to files
void savingThread() {
	while (true) {
		if (j >= i) {
			if (experiment_end == 1)
				break;
			usleep(10);
		} else {
			Dsave(j);
			RGBsave(j);

			j++;
		}
		//printf("i, j : %d , %d\n", i, j);
	}
}

// Initializes Kinect
void kinectInit(VideoCapture &capture) {
	cout << "Kinect/Xtion opening ..." << endl;
	capture.open(CV_CAP_OPENNI);

	if (!capture.isOpened()) {
		cout << "Cannot open a Xtion/Kinect" << endl;
		exit(-1);
	} else
		cout << "Xtion/Kinect opened" << endl;

	// Setting up 30Hz
	bool modeRes = false;
	modeRes = capture.set(CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE,
			CV_CAP_OPENNI_VGA_30HZ);

	if (!modeRes)
		cout
				<< "\nThis image mode is not supported by the device, the default value (CV_CAP_OPENNI_SXGA_15HZ) will be used.\n"
				<< endl;

	if (!capture.get(CV_CAP_OPENNI_IMAGE_GENERATOR_PRESENT)) {
		cout << "\nDevice doesn't contain image generator." << endl;
	}
}

int main(int argc, char* argv[]) {
	// Creating cyclic buffers
	rgbImg = new Mat [CYCLIC_BUFFER_SIZE];
	dMap = new Mat [CYCLIC_BUFFER_SIZE];
	cameraImg = new Mat [CYCLIC_BUFFER_SIZE];

	// Configuration file
	ifstream odczyt;
	odczyt.open("config.txt");

	string dummy, address;
	// comment + address
	std::getline(odczyt, dummy);
	std::getline(odczyt, address);
	odczyt.close();

	cout << "Parameters: " << address << endl;

	// TCP connection with computer responsible for synchronization

	int sockfd = 0;
	char recvbuf[1024];
	struct sockaddr_in serv_addr;

	// Creating socket
	if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		printf("\n Error : Could not create socket \n");
		return 1;
	}

	// Address
	memset(&serv_addr, '0', sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(3000);

	// ?
	if (inet_pton(AF_INET, address.c_str(), &serv_addr.sin_addr) <= 0) {
		printf("\n inet_pton error occured\n");
		return 1;
	}

	// Connecting
	if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr))
			< 0) {
		printf("\n Error : Connect Failed \n");
		return 1;
	}

	// Starting kinect connection
	VideoCapture capture;
	kinectInit(capture);

	// Starting saving thread
	std::thread t(savingThread);

	// Receiving buffer for TCP/IP communication

	printf("Ready for action!\n");
	string recvStr = "";

	//long long start = GetTickCount();
	while (true) {
		// Sending "READY" to server
		sprintf(recvbuf, "READY");
		write(sockfd, recvbuf, 5);

		// Receiving permission to capture data
		//		"START %d_"  || %d=='X' means stop
		int n = read(sockfd, recvbuf, sizeof(recvbuf)-1);
		printf("Received %d chars\n", n);

		// Checking if we didn't miss any synchronization packets
		recvStr = string(recvbuf);
		recvStr = recvStr.substr(6, n-8);
		if (n > 0)
			printf("Received number: %s, expected %d\n", recvStr.c_str(), i);

		// We got the stop
		if (recvStr.find('X') != string::npos)
			break;

		// Creating new MATs
		dMap[i % CYCLIC_BUFFER_SIZE] = Mat();
		rgbImg[i % CYCLIC_BUFFER_SIZE] = Mat();

		if (!capture.grab()) {
			cout << "Can not grab images." << endl;
			return -1;
		}

		// Retrieving data for saving
		capture.retrieve(dMap[i % CYCLIC_BUFFER_SIZE],
				CV_CAP_OPENNI_DEPTH_MAP);
		capture.retrieve(rgbImg[i % CYCLIC_BUFFER_SIZE],
				CV_CAP_OPENNI_BGR_IMAGE);

		// Increment buffer position
		i++;

	}

	// Release camera and w8 for save
	experiment_end = true;

	// Waiting for thread end
	t.join();

	// Closing
	capture.release();

	return 0;
}

