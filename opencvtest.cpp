#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main() 
{
	VideoCapture cap(1);

	if (!cap.isOpened()) 
	{ //check if video device has been initialised
		cout << "cannot open camera";
		return -1;
	}

	//parameter initializtion
	cap.set(CV_CAP_PROP_FPS, 30);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	//unconditional loop
	while(true)
	{
		Mat frame;
		cap.read(frame);
//		cvtColor(frame, grayframe, CV_BGR2GRAY); //transform to grayimage
		imshow("live", frame);
		if (waitKey(1) == 'q' ) break;
	}
	return 0;
}
