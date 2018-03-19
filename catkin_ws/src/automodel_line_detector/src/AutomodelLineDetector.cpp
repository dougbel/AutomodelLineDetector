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



		cv::imshow(IN_NAMED_WINDOW,image);

		///////////choosing pixels////

		//color
		//lower_yellow = np.array([20, 100, 100], dtype = “uint8”)
		//upper_yellow = np.array([30, 255, 255], dtype=”uint8")
		//mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

		//mask_yw = cv2.bitwise_or(mask_white, mask_yellow)
		//mask_yw_image = cv2.bitwise_and(gray_image, mask_yw)

		//gray scale
		Mat mask_white;
		Mat mask_yw_image;
		inRange(image, canny_lowThreshold, canny_highThreshold, mask_white);
		bitwise_and(image,mask_white, mask_yw_image);

		//erasing the horizon
		Rect region_of_interest = Rect(0, 0, image.cols, .01*canny_perBlindHorizon*image.rows);
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
        rho = hough_int_rho;
        theta = hough_int_theta*3.1416/180;

		HoughLines( mask_yw_image, linesLeft,rho, theta, hough_threshold,srn, stn, 0, 1.0472 );  //max 60 grad
		HoughLines( mask_yw_image, linesRight,rho, theta, hough_threshold,srn, stn, 2.0944, CV_PI);  //min 120 grad

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

		cv::waitKey(15);
	}

	void AutomodelLineDetector::createGUI() {
		cv::namedWindow(IN_NAMED_WINDOW);
		cv::namedWindow(OUT_NAMED_WINDOW);

		createTrackbar( "Threshold low", IN_NAMED_WINDOW, &canny_lowThreshold, 255);
		createTrackbar( "Threshold high", IN_NAMED_WINDOW, &canny_highThreshold, 255);
		createTrackbar( "Perc horizon", IN_NAMED_WINDOW, &canny_perBlindHorizon, 100);


		createTrackbar( "Hough ro", IN_NAMED_WINDOW, &hough_int_rho, 50);
		createTrackbar( "Hough theta", IN_NAMED_WINDOW, &hough_int_theta, 360);
		createTrackbar( "Hough threshold", IN_NAMED_WINDOW, &hough_threshold, 300);

	}

	void AutomodelLineDetector::readDefaultParameters() {
		canny_lowThreshold = 172;
		canny_highThreshold = 179;
		canny_perBlindHorizon = 46;

		hough_int_rho = 1;
		hough_int_theta =1;
		hough_threshold = 45;

		if (!nodeHandle.getParam("lowThreshold", canny_lowThreshold)) {
			ROS_ERROR("Could not find lowThreshold parameter!");
			//ros::requestShutdown();
		}
		if (!nodeHandle.getParam("highThreshold", canny_highThreshold)) {
			ROS_ERROR("Could not find highThreshold parameter!");
			//ros::requestShutdown();
		}
		if (!nodeHandle.getParam("canny_perBlindHorizon", canny_perBlindHorizon)) {
			ROS_ERROR("Could not find canny_perBlindHorizon parameter!");
			//ros::requestShutdown();
		}
		if (!nodeHandle.getParam("int_rho", hough_int_rho)) {
			ROS_ERROR("Could not find int_rho parameter!");
			//ros::requestShutdown();
		}
		if (!nodeHandle.getParam("int_theta", hough_int_theta)) {
				ROS_ERROR("Could not find int_theta parameter!");
				//ros::requestShutdown();
			}
		if (!nodeHandle.getParam("threshold", hough_threshold)) {
			ROS_ERROR("Could not find threshold parameter!");
			//ros::requestShutdown();
		}

		ROS_INFO_STREAM("Canny lowThreshold: "<< canny_lowThreshold);
		ROS_INFO_STREAM("Canny highThreshold: "<< canny_highThreshold);
		ROS_INFO_STREAM("Percentage blind horizon: "<< canny_perBlindHorizon);
		ROS_INFO_STREAM("Hough rho (distance accumulador): "<< hough_int_rho);
		ROS_INFO_STREAM("Hough theta (angle accumulator): "<< hough_int_theta);
		ROS_INFO_STREAM("Hough accumulator threshold: "<< hough_threshold);

	}



} /* namespace automodel */

