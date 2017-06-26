#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/video/tracking.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main() 
{
	VideoCapture cap(0);

	if (!cap.isOpened()) 
	{ //check if video device has been initialised
		cout << "cannot open camera";
		return -1;
	}

	//parameter initializtion
	cap.set(CV_CAP_PROP_FPS, 30);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 720);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	int frame = 1;
	vector<Point2f> corners, nextcorners;
	Mat prevgrayimage;

	while(true)
	{
		if (frame == 1)
		{
			//feature extraction
			Mat image, grayimage;
			cap.read(image);
			cvtColor(image, grayimage, CV_BGR2GRAY); //transform to grayimage

			//corner detection
			int maxCorners = 20;
			double qualityLevel = 0.01;
			double minDistance = 10;
			int blockSize = 3;
			bool useHarrisDetector = false;
			double k = 0.04;
			goodFeaturesToTrack(grayimage, corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k);
			
			////refine corner detection
			//cornerSubPix(grayimage, corners, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.1));
			
			//update
			prevgrayimage = grayimage;

			//plot feature point
			for (size_t i = 0; i < corners.size(); i++)
			{
				circle(image, corners[i], 20, cv::Scalar(255.), -1);
			}

			namedWindow("tracking", WINDOW_NORMAL); // Create a window for display.	
			imshow("tracking", image);                // Show our image inside it.
			if (waitKey(1) == 'q') break;
		}
		else
		{
			Mat nextimage, nextgrayimage;
			cap.read(nextimage);
			cvtColor(nextimage, nextgrayimage, CV_BGR2GRAY); //transform to grayimage

			//opticalflow
			vector<uchar> status;
			vector<float> err;
			calcOpticalFlowPyrLK(prevgrayimage, nextgrayimage, corners, nextcorners, status, err);

			//update
			prevgrayimage = nextgrayimage;
			corners = nextcorners;

			//plot feature point
			for (size_t i = 0; i < nextcorners.size(); i++)
			{
				circle(nextimage, nextcorners[i], 5, cv::Scalar(255.), -1);
			}

			imshow("tracking", nextimage);                // Show our image inside it.
			if (waitKey(1) == 'q') break;

		}
		frame++;
	}
	return 0;
}
