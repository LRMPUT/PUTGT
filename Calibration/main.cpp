#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

int main()
{
	// Opening Xtion/Kinect to save calibration files
	cv::VideoCapture captureKinect(CV_CAP_OPENNI);
	if (captureKinect.isOpened())
		std::cout<<"Kinect v1 opened!"<<std::endl;
	else
		exit(0);
	

	// Supported commands
	std::cout<<"'q' - quit \t 'c' - take picture"<< std::endl;

	int index = 0;
	bool running = true;
	while(running)
	{		
		cv::Mat rgbImage;

		while(running)
		{	
			// Grab Image
			if( !captureKinect.grab() ) ;
		
			// Get that rgb Image	
			captureKinect.retrieve( rgbImage, CV_CAP_OPENNI_BGR_IMAGE );
			
			// Show it
			cv::imshow("rgb Image", rgbImage);	

			// Wait for instructions
			char c = cv::waitKey(1);
			if ( c == 'c') break;
			if ( c == 'q'){
				running = false;
				break;			
			};
		}
		

		// Save to file
		char nazwa[30];
		sprintf(nazwa,"calibrationKinect_%03d.png",index);
		cv::imwrite(nazwa,rgbImage);
	
		// Just some debug information
		std::cout<<"Saved calibration numer " << index << " !" << std::endl;
		
		// Increment the index
		index++;
	}

	// Release interface
	captureKinect.release();
}
