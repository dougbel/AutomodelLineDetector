/*
 * AutomodelLineDetector.h
 *
 *  Created on: Mar 16, 2018
 *      Author: dougbel
 */

#ifndef SRC_AUTOMODELLINEDETECTOR_H_
#define SRC_AUTOMODELLINEDETECTOR_H_

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <iostream>

using namespace std;
using namespace cv;

namespace automodel {


	const string IN_NAMED_WINDOW = "Input";
	const string OUT_NAMED_WINDOW = "Output";
	const string OUT2_NAMED_WINDOW = "Output prev";

	class AutomodelLineDetector {
		public:
			AutomodelLineDetector(ros::NodeHandle& nodeHandle);
			virtual ~AutomodelLineDetector();

			void detect(const sensor_msgs::ImageConstPtr& );

			void createGUI();
			void readDefaultParameters();



		private:
			ros::NodeHandle	nodeHandle;
			//Para Cannny
			int canny_lowThreshold;
			int canny_highThreshold;
			int canny_perBlindHorizon;

			//Para Hough
			double rho;
			double theta;
			int hough_int_rho;
			int hough_int_theta;
			int hough_threshold;
	};

}

#endif
