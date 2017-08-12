// plot circle on image
	Point corner;
	corner.x = 200;
	corner.y = 200;
	circle(image, corner, 10, cv::Scalar(255.), -1);
	imshow("featuremap", image);                // Show our image inside it.
	waitKey(0); // Wait for a keystroke in the window

///////////////////////////////////////////////////////////////////////////////////
// real-time capture
	videocapture cap(0);

	if (!cap.isopened()) 
	{ //check if video device has been initialised
		cout << "cannot open camera";
		return -1;
	}

	//parameter initializtion
	cap.set(cv_cap_prop_fps, 30);
	cap.set(cv_cap_prop_frame_width, 640);
	cap.set(cv_cap_prop_frame_height, 480);

	//unconditional loop
	while(true)
	{
		mat frame;
		cap.read(frame);
//		cvtcolor(frame, grayframe, cv_bgr2gray); //transform to grayimage
		imshow("live", frame);
		if (waitkey(1) == 'q' ) break;
	}
	return 0;	