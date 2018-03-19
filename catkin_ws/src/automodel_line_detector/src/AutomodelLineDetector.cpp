/*
 * AutomodelLineDetector.cpp
 *
 *  Created on: Mar 16, 2018
 *      Author: dougbel
 */

#include "automodel_line_detector/AutomodelLineDetector.h"


namespace automodel {


	cv::Mat detectedEdges;

	AutomodelLineDetector::AutomodelLineDetector(ros::NodeHandle& nodeHandle_) :
			nodeHandle(nodeHandle_) {

		 // subsribe topic
		  ros::Subscriber sub = nodeHandle.subscribe("/app/camera/rgb/image_raw", 1000,
					&AutomodelLineDetector::detect, this);

		  readDefaultParameters();
		  createGUI();



		  // publish
//		  image_transport::ImageTransport it(nodeHandle);
//
//		  image_transport::Publisher pub = it.advertise("camera/lines", 1);
//
//		  sensor_msgs::ImagePtr msg;
//
//		  ros::Rate loop_rate(5);

//
//		  int  c;
//		  while (nodeHandle.ok()) {
//
//			// Check if grabbed frame is actually full with some content
//			if(!detectedEdges.empty()) {
//			  msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", detectedEdges).toImageMsg();
//			  pub.publish(msg);
//			  cv::imshow(OUT_NAMED_WINDOW,detectedEdges);
//			  c=cv::waitKey(1);
//			  if(c=='s'){
//				  imwrite( "Gray_Image.jpg", detectedEdges );
//			  }
//			}
//
//
//			ros::spinOnce();
//			loop_rate.sleep();
//		  }

		  ros::spin();

	}

	AutomodelLineDetector::~AutomodelLineDetector() {
		cv::destroyWindow(IN_NAMED_WINDOW);
		cv::destroyWindow(OUT_NAMED_WINDOW);
		ROS_INFO_STREAM("Destroying Automodel Line Detector");
	}

	void AutomodelLineDetector::detect(
		const sensor_msgs::ImageConstPtr& msg) {

		cv::Mat imageColor = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::Mat image = cv_bridge::toCvShare(msg, "mono8")->image;

		//	cv::Mat gray;
		//	Convert the image to grayscale
		//	cv::cvtColor( image, gray, CV_BGR2GRAY );

		cv::imshow(IN_NAMED_WINDOW,image);

		//choosing pixels
		Mat mask_white;
		Mat mask_yw_image;
		inRange(image, lowThreshold, highThreshold, mask_white);
		bitwise_and(image,mask_white, mask_yw_image);

		//erasing the horizon
		Rect region_of_interest = Rect(0, 0, image.cols, .01*perBlindHorizon*image.rows);
		Mat image_roi = mask_yw_image(region_of_interest);
		image_roi= cv::Mat::zeros(image_roi.size(), image_roi.type());


		int low_threshold = 50;
		int high_threshold = 150;
		int kernel_size = 3;
		cv::Canny(mask_yw_image,mask_yw_image,low_threshold,high_threshold);




        double srn=0;
        double stn =0;
        vector<Vec2f> linesRight;
        vector<Vec2f> linesLeft;
        rho = int_rho;
        theta = int_theta*3.1416/180;
		HoughLines( mask_yw_image, linesLeft,rho, theta, threshold,srn, stn, 0, 1.0472 );  //max 60 grad
		HoughLines( mask_yw_image, linesRight,rho, theta, threshold,srn, stn, 2.0944, CV_PI);  //min 120 grad

		if(linesRight.size()>0){
			float rhot = linesRight[0][0], thetat = linesRight[0][1];
			Point pt1, pt2;
			double a = cos(thetat), b = sin(thetat);
			double x0 = a*rhot, y0 = b*rhot;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			line( imageColor, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
			putText(imageColor, "RIGTH: rho "+to_string(rhot)+" angle: "+to_string(thetat), cvPoint(30,30),
			    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,250), 1, CV_AA);
		}
		if(linesLeft.size()>1){
			float rhot = linesLeft[1][0], thetat = linesLeft[1][1];
			Point pt1, pt2;
			double a = cos(thetat), b = sin(thetat);
			double x0 = a*rhot, y0 = b*rhot;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			line( imageColor, pt1, pt2, Scalar(0,255,0), 3, CV_AA);
			putText(imageColor, "LEFT: rho "+to_string(rhot)+" angle: "+to_string(thetat),cvPoint(30,80),
			    FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,250,0), 1, CV_AA);
		}




		cv::imshow(OUT2_NAMED_WINDOW,imageColor);
		cv::imshow(OUT_NAMED_WINDOW,mask_yw_image);

	//	cv::GaussianBlur( image, image, cv::Size( 21, 21 ), 5, 5 );
	//
	//	cv::Mat gray;
	//	/// Convert the image to grayscale
	//	cv::cvtColor( image, gray, CV_BGR2GRAY );
	//
	//	int lowThreshold=10;
	//	int ratio = 3;
	//	int kernel_size = 3;
	//
	//	cv::Canny( gray, detectedEdges, lowThreshold, lowThreshold*ratio, kernel_size );

		cv::waitKey(15);
	}

	void AutomodelLineDetector::createGUI() {
		cv::namedWindow(IN_NAMED_WINDOW);
		cv::namedWindow(OUT_NAMED_WINDOW);

		createTrackbar( "Threshold low", IN_NAMED_WINDOW, &lowThreshold, 255);
		createTrackbar( "Threshold high", IN_NAMED_WINDOW, &highThreshold, 255);
		createTrackbar( "Perc horizon", IN_NAMED_WINDOW, &perBlindHorizon, 100);


		createTrackbar( "Hough ro", IN_NAMED_WINDOW, &int_rho, 50);

		createTrackbar( "Hough theta", IN_NAMED_WINDOW, &int_theta, 360);
		createTrackbar( "Threshold", IN_NAMED_WINDOW, &threshold, 300);

	}

	void AutomodelLineDetector::readDefaultParameters() {
		lowThreshold = 172;
		highThreshold = 179;
		perBlindHorizon = 46;

		int_rho = 1;
		int_theta =1;
		threshold = 45;

	}



} /* namespace automodel */

