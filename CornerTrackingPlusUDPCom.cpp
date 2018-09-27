#include <iostream>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/video/tracking.hpp"
#include "boost/asio.hpp"

using namespace cv;
using namespace std;
using namespace boost::asio;

void CalcPosAtd(const vector<Point2f>& inputCorners, double* posAtd)
{
	const double Pi = 3.14159265358979323846;
	posAtd[0] = inputCorners[0].x;
	posAtd[1] = inputCorners[0].y;
	double vectorY = inputCorners[1].y - inputCorners[0].y;
	double vectorX = inputCorners[1].x - inputCorners[0].x;
	posAtd[2] = asin(vectorY / sqrt(vectorY*vectorY + vectorX*vectorX));
	if (vectorX < 0)
	{
		if (vectorY > 0)
			posAtd[2] = Pi - posAtd[2];
		else
			posAtd[2] = -Pi - posAtd[2];
	}
	// coordinates trasnformation from camera to inertia
	posAtd[2] = -posAtd[2];
}

int main()
{
	//create udp server
	io_service ios;
	ip::udp::endpoint server_ep(ip::udp::v4(), 8112);
	ip::udp::socket sock(ios, server_ep);

	//define udp client
	ip::udp::endpoint send_ep(ip::address::from_string("127.0.0.1"), 9111);

	// camera 
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
	vector<Point2f> corners, cornersToTrack, nextcorners;
	Mat prevgrayimage;

	while (true)
	{
		if (frame == 1)
		{
			//feature extraction
			Mat image, grayimage;
			cap.read(image);
			cvtColor(image, grayimage, CV_BGR2GRAY); //transform to grayimage

			//corner detection
			int maxCorners = 20;
			double qualityLevel = 0.1;
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
				circle(image, corners[i], 2, cv::Scalar(255.), -1);
				putText(image, to_string(i), corners[i], FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0),1);
			}

			namedWindow("tracking", WINDOW_NORMAL); // Create a window for display.	
			imshow("tracking", image);                // Show our image inside it.
			cout << "want to continue? " << endl;
			char inputFlag='n';
			if (waitKey(1) == 'q') break;

			// select the corners to track
			int indexCornersToTrack[2];
			cout << "enter the corner index" << endl;
			for (int i = 0;i < 2;++i)
			{
				cin >> indexCornersToTrack[i];
			}
			for (int i = 0;i < 2;++i)
			{
				cornersToTrack.push_back(corners[indexCornersToTrack[i]]);
			}
		}
		else
		{
			Mat nextimage, nextgrayimage;
			cap.read(nextimage);
			cvtColor(nextimage, nextgrayimage, CV_BGR2GRAY); //transform to grayimage

			//sparse optical flow tracking
			vector<uchar> status;
			vector<float> err;
			calcOpticalFlowPyrLK(prevgrayimage, nextgrayimage, cornersToTrack, nextcorners, status, err);

			//update
			prevgrayimage = nextgrayimage;
			cornersToTrack = nextcorners;

			// calculate position and attitude
			double posAtd_send[3] = { 0, 0, 0 };
			CalcPosAtd(cornersToTrack, posAtd_send);

			//plot feature point
			for (size_t i = 0; i < nextcorners.size(); i++)
			{
				circle(nextimage, nextcorners[i], 2, cv::Scalar(255.), -1);
				if (i==0)
					putText(nextimage, 
						to_string(posAtd_send[0])+ " " + to_string(posAtd_send[1])+ " " + to_string(posAtd_send[2]),
						Point2f(200,200),
						FONT_HERSHEY_SIMPLEX, 
						0.5, 
						CV_RGB(255, 0, 0), 1);
			}

			imshow("tracking", nextimage);                // Show our image inside it.
			if (waitKey(1) == 'q') break;

			// UDP connection
			vector<char> v(5, 0);
			ip::udp::endpoint remote_ep;

			// check whether there is data available in buffer
			boost::asio::socket_base::bytes_readable command(true);
			sock.io_control(command);
			std::size_t bytes_readable = command.get();

			// only read data when available, ; therefore to prevent socket blocking
			if (bytes_readable > 0)
			{
				sock.receive_from(buffer(v), remote_ep);
				if (v[0] == 'y')
				{
					sock.send_to(buffer(posAtd_send), send_ep);
				}
			}
		}
		frame++;
	}
	return 0;
}
