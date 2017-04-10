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

Mat *dMap;
Mat *rgbImg;
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
	sprintf(zapis, "rgb_%05d.png", k);
	imwrite(zapis, rgbImg[k % CYCLIC_BUFFER_SIZE]);
}

// Thread responsible for saving data to files
void savingThread() {
	while (true) {
		if (j >= i) {
			if (experiment_end == 1)
				break;
			//usleep(10);
		} else {
			Dsave(j);
			RGBsave(j);

			j++;
		}
		//printf("i, j : %d , %d\n", i, j);
	}
}


int main()
{
    // Creating cyclic buffers
    rgbImg = new Mat [CYCLIC_BUFFER_SIZE];
    dMap = new Mat [CYCLIC_BUFFER_SIZE];
   
    // Starting saving thread
    std::thread t(savingThread);

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

    while(true) {
            dev->wait_for_frames();

            // Creating OpenCV Matrix from a color image
	    Mat color(Size(640, 480), CV_8UC3, (void*)dev->get_frame_data(rs::stream::color), Mat::AUTO_STEP);
	    rgbImg[i % CYCLIC_BUFFER_SIZE] = color;

 	    Mat depth(Size(640, 480), CV_16UC1, (void*)dev->get_frame_data(rs::stream::depth), Mat::AUTO_STEP);
	    dMap[i % CYCLIC_BUFFER_SIZE] = depth;

	    i++;
    }
    // Release camera and w8 for save
    experiment_end = true;

    // Waiting for thread end
    t.join();

    delete [] rgbImg;
    delete [] dMap;

    return 0;
}
