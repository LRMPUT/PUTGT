// include the librealsense C++ header file
#include <librealsense/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

// C++11: threads
//#include <unistd.h>
#include <thread>

#define CYCLIC_BUFFER_SIZE 5000

using namespace std;
using namespace cv;



int main()
{
    // Create a context object. This object owns the handles to all connected realsense devices
    rs::context ctx;

    // Access the first available RealSense device
    rs::device * dev = ctx.get_device(0);

    // Configure Infrared stream to run at VGA resolution at 30 frames per second
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 30);
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::bgr8, 30);
    // Start streaming
    dev->start();

    dev->set_option(rs::option::r200_lr_auto_exposure_enabled, 1);

    // Camera warmup - Dropped several first frames to let auto-exposure stabilize
    for(int i = 0; i < 30; i++)
       dev->wait_for_frames();

    int i=0;

    Mat color;
    bool running = true;
    while(true) {
	    while(running)
	    {
		    dev->wait_for_frames();

		    // Creating OpenCV Matrix from a color image
		    color = Mat(Size(640, 480), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color), Mat::AUTO_STEP);
		   
		    cv::imshow("rgb Image", color);	

		    // Wait for instructions
		    char c = cv::waitKey(1);
		    if ( c == 'c' || c == 27) break;
		    if ( c == 'q') {
			   running = false;
			   break;			
		    };
	    }

	    if (!running)
			break;
 	    
	    char zapis[40];
            sprintf(zapis, "rgb_%05d.png", i);
	    imwrite(zapis, color);

	    i++;
    }

    return 0;
}
