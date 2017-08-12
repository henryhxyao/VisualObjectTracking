// Pylon_with_OpenCV.cpp

/*
    Note: Before getting started, Basler recommends reading the Programmer's Guide topic
    in the pylon C++ API documentation that gets installed with pylon.
    If you are upgrading to a higher major version of pylon, Basler also
    strongly recommends reading the Migration topic in the pylon C++ API documentation.

    This sample illustrates how to grab and process images using the CInstantCamera class and OpenCV.
    The images are grabbed and processed asynchronously, i.e.,
    while the application is processing a buffer, the acquisition of the next buffer is done
    in parallel.
	
	OpenCV is used to demonstrate an image display, an image saving and a video recording.

    The CInstantCamera class uses a pool of buffers to retrieve image data
    from the camera device. Once a buffer is filled and ready,
    the buffer can be retrieved from the camera object for processing. The buffer
    and additional image data are collected in a grab result. The grab result is
    held by a smart pointer after retrieval. The buffer is automatically reused
    when explicitly released or when the smart pointer object is destroyed.
*/

#include "stdafx.h"

// Include files to use OpenCV API.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using OpenCV objects.
using namespace cv;

// Namespace for using cout.
using namespace std;

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Create an instant camera object with the camera device found first.
        CInstantCamera camera( CTlFactory::GetInstance().CreateFirstDevice());

		// Get a camera nodemap in order to access camera parameters.
		GenApi::INodeMap& nodemap= camera.GetNodeMap();

		// Open the camera before accessing any parameters.
		camera.Open();
		// Create pointers to access the camera Width and Height parameters.
		GenApi::CIntegerPtr width= nodemap.GetNode("Width");
		GenApi::CIntegerPtr height= nodemap.GetNode("Height");

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        //camera.MaxNumBuffer = 5;

		// Create a pylon ImageFormatConverter object.
		CImageFormatConverter formatConverter;
		// Specify the output pixel format.
		formatConverter.OutputPixelFormat= PixelType_BGR8packed;
		// Create a PylonImage that will be used to create OpenCV images later.
		CPylonImage pylonImage;

		// Create an OpenCV image.
		Mat openCvImage;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
		camera.StartGrabbing(GrabStrategy_LatestImageOnly);
	
        // This smart pointer will receive the grab result data.
        CGrabResultPtr ptrGrabResult;

		// tracking parameter 
		int frame = 1;
		vector<Point2f> corners, nextcorners;
		Mat prevgrayimage;

        // Camera.StopGrabbing() is called automatically by the RetrieveResult() method
        // when c_countOfImagesToGrab images have been retrieved.
        while ( camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                //cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                //cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;

				// Convert the grabbed buffer to a pylon image.
				formatConverter.Convert(pylonImage, ptrGrabResult);

				// Create an OpenCV image from a pylon image.
				openCvImage= cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

				// Create an OpenCV display window.
				namedWindow( "OpenCV Display Window", CV_WINDOW_NORMAL); // other options: CV_AUTOSIZE, CV_FREERATIO
				
				// Display the current image in the OpenCV display window.
				imshow( "OpenCV Display Window", openCvImage);
				// Define a timeout for customer's input in ms.
				// '0' means indefinite, i.e. the next image will be displayed after closing the window. 
				// '1' means live stream

				// optical flow tracking algorithm
				if (frame == 1)
				{
					//feature extraction
					Mat image, grayimage;
					image = openCvImage;
					cvtColor(image, grayimage, CV_BGR2GRAY); //transform to grayimage

					//corner detection
					int maxCorners = 10;
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
						circle(image, corners[i], 5, cv::Scalar(255.), -1);
					}

					namedWindow("tracking", WINDOW_AUTOSIZE); // Create a window for display.	
					imshow("tracking", image);                // Show our image inside it.
					waitKey(1);
					//if (waitKey(1) == 'q') break;
				}
				else
				{
					Mat nextimage, nextgrayimage;
					nextimage = openCvImage;
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
					waitKey(1); 
					// if (waitKey(1) == 'q') break;    

				}    
				// waitKey(1);
				frame++;

#ifdef PYLON_WIN_BUILD
#endif
            }
            else
            {
                cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
            }
        }

    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred." << endl
        << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Comment the following two lines to disable waiting on exit.
    cerr << endl << "Press Enter to exit." << endl;
    while( cin.get() != '\n');

    return exitCode;
}
