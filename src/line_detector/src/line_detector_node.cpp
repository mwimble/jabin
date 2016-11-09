#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "line_detector/MazeDetector.h"
#include "line_detector/line_detector.h"

#include <iostream>
#include <string>

using namespace cv;
using namespace std;

// Scalar colorArray[] = {Scalar(0, 0, 255),Scalar(255, 0, 0),Scalar(0, 255, 0),Scalar(255, 128, 64),Scalar(64, 128, 255),Scalar(128, 255, 64),Scalar(77,77,77),Scalar(164,124,68),Scalar(204,196,132),Scalar(164,148,147),Scalar(163,123,67),Scalar(26,122,26), Scalar(195,195,50),Scalar(193,193,193),Scalar(255,248,73),Scalar(243,243,243)};

static const std::string OPENCV_WINDOW = "Image window";
Scalar colorArray[] = {Scalar(0, 0, 255),Scalar(255, 0, 0),Scalar(0, 255, 0),Scalar(255, 128, 64),Scalar(64, 128, 255),Scalar(128, 255, 64),Scalar(77,77,77),Scalar(164,124,68),Scalar(204,196,132),Scalar(164,148,147),Scalar(163,123,67),Scalar(26,122,26), Scalar(195,195,50),Scalar(193,193,193),Scalar(255,248,73),Scalar(243,243,243)};

bool saveImages = false;
bool showImage = false;

class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher detectPublisher;
  
public:
  ImageConverter()
    : it_(nh_) {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/line_camera", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
	detectPublisher = nh_.advertise<line_detector::line_detector>("/lineDetect", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter() {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
      //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    	Mat originalImage = cv_ptr->image;
    	// ROS_INFO("originalImage.cols: %d, originalImage.rows: %d", originalImage.cols, originalImage.rows);
    	MazeDetector mazeDetector(originalImage);
        mazeDetector.kmeansImage();
        mazeDetector.detectLines();

        line_detector::line_detector ld;

        static int nextSequence = 0;
        const int minAcceptableHorizontalLineLength = 120;
        ros::Time currentTime = ros::Time::now();
        Size size = originalImage.size();

        ld.sequence = nextSequence++;
        ld.cameraWidth = size.width;
        ld.cameraHeight = size.height;

        ld.horizontalLowerLeftY = mazeDetector.horizontalLowerLeftY;
        ld.horizontalUpperRightX = mazeDetector.horizontalUpperRightX;
        ld.horizontalUpperRightY = mazeDetector.horizontalUpperRightY;
        ld.horizontalCurveA = mazeDetector.horizontalCurve.a;
        ld.horizontalCurveB = mazeDetector.horizontalCurve.b;

        ld.verticalLowerLeftX = mazeDetector.verticalLowerLeftX;
        ld.verticalLowerLeftY = mazeDetector.verticalLowerLeftY;
        ld.verticalUpperRightX = mazeDetector.verticalUpperRightX;
        ld.horizontalLowerLeftX = mazeDetector.horizontalLowerLeftX;
        ld.verticalUpperRightY = mazeDetector.verticalUpperRightY;
        ld.verticalCurveA = mazeDetector.verticalCurve.a;
        ld.verticalCurveB = mazeDetector.verticalCurve.b;

        ld.horizontalLeft = mazeDetector.horizontalLowerLeftX;
        ld.horizontalBottom = (mazeDetector.horizontalCurve.b * mazeDetector.horizontalLowerLeftX) + mazeDetector.horizontalCurve.a;
        ld.horizontalLength = mazeDetector.horizontalUpperRightX - mazeDetector.horizontalLowerLeftX;
        ld.horizontalToLeft = (mazeDetector.horizontalLowerLeftX < int(size.width * 0.25)) && (ld.horizontalLength > minAcceptableHorizontalLineLength);
        ld.horizontalToRight = (mazeDetector.horizontalUpperRightX > int(size.width * 0.75)) && (ld.horizontalLength > minAcceptableHorizontalLineLength);

        ld.verticalBottom = mazeDetector.verticalLowerLeftY;
        ld.verticalLeft = (mazeDetector.verticalCurve.b * mazeDetector.verticalLowerLeftY) + mazeDetector.verticalCurve.a;
        ld.verticalYlength = mazeDetector.verticalLowerLeftY - mazeDetector.verticalUpperRightY;
        ld.verticalToBottom = mazeDetector.verticalLowerLeftY > int(size.height * 0.75);
        ld.verticalToTop = mazeDetector.verticalUpperRightY < (ld.horizontalBottom - 40);
        ld.verticalIntercept = ld.verticalCurveA + (ld.verticalCurveB * size.width);

        ld.header.stamp = currentTime;
        detectPublisher.publish(ld);

        if (saveImages || showImage) {
	        Mat ti = mazeDetector.getOriginalImage();

	        if (mazeDetector.verticalLowerLeftY != -1) {
	        	cv::rectangle(ti, 
	        			 Point(mazeDetector.verticalLowerLeftX, mazeDetector.verticalLowerLeftY),
	        			 Point(mazeDetector.verticalUpperRightX, mazeDetector.verticalUpperRightY), colorArray[0], 3, 8);
	        }

	        if (mazeDetector.horizontalLowerLeftY != -1) {
	        	cv::rectangle(ti, 
	        			 Point(mazeDetector.horizontalLowerLeftX, mazeDetector.horizontalLowerLeftY),
	        			 Point(mazeDetector.horizontalUpperRightX, mazeDetector.horizontalUpperRightY), colorArray[1], 3, 8);
	        }

	        Scalar axisColor = Scalar(0, 255, 255);
	        Scalar verticalLineColor = Scalar(0, 136, 255);
            cv::line(ti, Point(size.width / 2, size.height), Point (size.width / 2, 0), axisColor, 1, 8);
            cv::line(ti, Point(0, size.height / 2), Point(size.width, size.height / 2), axisColor, 1, 8);
	        cv::line(ti, Point(mazeDetector.verticalCurve.b * size.height + mazeDetector.verticalCurve.a, size.height), Point(mazeDetector.verticalCurve.a, 0), verticalLineColor, 1, 8);
	        cv::line(ti, Point(0, mazeDetector.horizontalCurve.a), Point(size.width, mazeDetector.horizontalCurve.b * size.width + mazeDetector.horizontalCurve.a), verticalLineColor, 1, 8);

	        if (showImage) {
		        imshow("Scaled Original Image", mazeDetector.getOriginalImage());
		        imshow("RGB Image", mazeDetector.getRgbImage());
		    }

			if (saveImages) {
				ros::Time currentTime = ros::Time::now();
				double secs = currentTime.toSec();
				char fn[128];
				// sprintf(fn, "/home/pi/images/%-20.9f-ORIG.jpg", secs);
				// imwrite(fn, mazeDetector.getOriginalImage());
				sprintf(fn, "/home/pi/images/%-20.9f-PROC-%d.jpg", secs, ld.sequence);
				imwrite(fn, mazeDetector.getRgbImage());
			}
	    }
   }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main( int argc, char** argv ) {
	ros::init(argc, argv, "line_detector_node");
	ros::NodeHandle nh("~");
	nh.getParam("save_images", saveImages);
	ROS_INFO("[line_detector_node] PARAM save_images: %s", saveImages ? "TRUE" : "false");
	nh.getParam("show_image", showImage);
	ROS_INFO("[line_detector_node] PARAM show_image: %s", showImage ? "TRUE" : "false");
	// ros::Publisher detectPublisher = nh.advertise<line_detector::line_detector>("/lineDetect", 1);
	ROS_INFO("[line_detector_node] Starting to spin...");

    // VideoCapture cap(0);
    // if (!cap.isOpened()) {
    // 	ROS_ERROR("[line_detector_node] Unable to open VideoCapture device");
    //  	return -1;
    // }
    
 //    MazeDetector camera = MazeDetector(cap);

	// const int minAcceptableHorizontalLineLength = 120;
	
	ros::Rate r(20);

	ImageConverter ic;
	
	while (ros::ok()) {
		try {
			ros::spinOnce();
			r.sleep();
		} catch(...) {
		    ROS_ERROR("[line_detector_node] Unhandled exception");
        }
	}

	return 0;
}