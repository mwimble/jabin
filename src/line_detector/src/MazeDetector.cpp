#include "ros/ros.h"
#include "line_detector/MazeDetector.h"

#include <cv.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>   
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

using namespace cv;
using namespace std;

static double IMAGE_COLS = 320.0; // Scale image to be this many colums across

void ProccTimePrint(struct timespec start , string msg) {
	struct timespec end;
	clock_gettime(CLOCK_MONOTONIC, &end);
	double duration = ((double)(end.tv_sec - start.tv_sec) * 1.0e9 +
              (double)(end.tv_nsec - start.tv_nsec)) / 1.0e9;

	printf("%s %.4lf(sec) / %.4lf(fps) \n", msg.c_str(),  duration, 1.0 /duration );
	fflush(stdout);
	cout << "- - -" << endl;
} 

void handleControlChange(int newValue, void* userData) {
	// MazeDetector* camera = (MazeDetector*) userData;
	// cout << "New Value" << newValue << ", low hue value: " << camera->getLowHueThreshold() << endl;
}

MazeDetector::MazeDetector(Mat originalImage)
	: imageLoaded(false), morphSize(5,5), scaleFactor(1.0), originalImage_(originalImage) {
	nh_ = ros::NodeHandle("~");
	nh_.getParam("debug", debug_);
	//ROS_INFO("[MazeDetector] PARAM debug: %s", debug_ ? "TRUE" : "false");
	controlWindowName = "Ianthe Camera Controls";
    // KMEANS takes too long, even on a Jetson, so reduce the image resolution to make it faster.
    scaleFactor = IMAGE_COLS / originalImage_.cols;
	scaleOriginalImage(scaleFactor);
    // ROS_INFO("[MazeDetector] video capture scaleFactor: %5.3f, original cols: %d, original rows: %d, reshaped_image32f cols: %d, reshaped_image32f rows: %d",
    // 		 scaleFactor, 
    // 		 originalImage_.cols, 
    // 		 originalImage_.rows,
    // 		 reshaped_image32f.cols,
    // 		 reshaped_image32f.rows);
}

MazeDetector::MazeDetector(const std::string testFileName)
	: imageLoaded(false), morphSize(8,8), scaleFactor(1.0) {
	struct stat buffer;

	controlWindowName = "Ewyn Camera Controls";
	fileName = testFileName;
	if (stat (testFileName.c_str(), &buffer) == 0) {
		originalImage_ = imread(fileName);
		imageLoaded = !originalImage_.empty();

	    // KMEANS takes too long, even on a Jetson, so reduce the image resolution to make it faster.
	    scaleFactor = IMAGE_COLS / originalImage_.cols;

		if (imageLoaded) {
			scaleOriginalImage(scaleFactor);
		}
	} else {
		imageLoaded = false;
	}
}

MazeDetector::~MazeDetector() {
	fileName = "";
}

// Line configurations
//	...+++...	Vertical line
//  ++++++...	Left turn
//  ...++++++	Right turn
//  +++++++++	Left and right turn
void MazeDetector::detectLines() {
	if (grey_image.empty()) {
		throw new std::string("[MazeDetector::detectLines] No image to use");
	}

	int startOfContiguousColumnsOfWhitePixels = -1;
	std::vector<TLINE_SEGMENT> verticalLineSegments;
	int lengthOfContiguousColumnsOfWhitePixels;

	assert(grey_image.type() == CV_8UC1);
	// for (int row = grey_image.rows - 1; row >= (grey_image.rows / 2); row--) {
	int row = grey_image.rows - 1;
	{
		lengthOfContiguousColumnsOfWhitePixels = 0;
		for (int col = 0; col < grey_image.cols; col++) {
			uchar pixel = grey_image.at<uchar>(row, col);
			if (pixelRepresentsALine(pixel)) {
				if (startOfContiguousColumnsOfWhitePixels == -1) {
					// Capture the start only once.
					startOfContiguousColumnsOfWhitePixels = col;
					lengthOfContiguousColumnsOfWhitePixels = 1;
					if (debug_) {
						ROS_INFO("[MazeDetector::detectLines] row: %d, col: %d, VSTART startOfContiguousColumnsOfWhitePixels: %d, lengthOfContiguousColumnsOfWhitePixels: %d",
								  row,
								  col,
								  startOfContiguousColumnsOfWhitePixels,
								  lengthOfContiguousColumnsOfWhitePixels);
					}
				} else {
					// This is a continuation of the segment.
					lengthOfContiguousColumnsOfWhitePixels++;
					// if (debug_) {
					// 	ROS_INFO("[MazeDetector::detectLines] row: %d, col: %d, VCONT startOfContiguousColumnsOfWhitePixels: %d, lengthOfContiguousColumnsOfWhitePixels: %d",
					// 			  row,
					// 			  col,
					// 			  startOfContiguousColumnsOfWhitePixels,
					// 			  lengthOfContiguousColumnsOfWhitePixels);
					// }
				}
			} else if (startOfContiguousColumnsOfWhitePixels != -1) {
				// End of a contiguous line of pixels.
				evaluateLine(startOfContiguousColumnsOfWhitePixels, row, col, lengthOfContiguousColumnsOfWhitePixels);
				return;
			}
		} // for (int col...)
	} // for (int row)...

	evaluateLine(startOfContiguousColumnsOfWhitePixels, row, grey_image.cols - 1, lengthOfContiguousColumnsOfWhitePixels);
}

void MazeDetector::evaluateLine(int startOfContiguousColumnsOfWhitePixels, int row, int col, int lengthOfContiguousColumnsOfWhitePixels) {
	static const int expectedLineSize = 50 * scaleFactor;
	
	if (debug_) {
		ROS_INFO("[MazeDetector::detectLines]  row: %d, col: %d, VEND lengthOfContiguousColumnsOfWhitePixels: %d, expectedLineSize: %d",
				 row,
				 col,
				 lengthOfContiguousColumnsOfWhitePixels, 
				 expectedLineSize);
	}

	int midpoint;

	if ((lengthOfContiguousColumnsOfWhitePixels >= (expectedLineSize * 0.8)) &&
		(lengthOfContiguousColumnsOfWhitePixels <= (expectedLineSize * 1.2))) {
		// Seems like a part of a vertical line stripe.
		horizontalLowerLeftX = -1;
		horizontalLowerLeftY = -1;
		horizontalUpperRightX = -1;
		horizontalUpperRightY = -1;

		verticalLowerLeftY = row;
		verticalUpperRightY = row;
		verticalLowerLeftX = startOfContiguousColumnsOfWhitePixels;
		verticalUpperRightX = col;
		midpoint = startOfContiguousColumnsOfWhitePixels + (lengthOfContiguousColumnsOfWhitePixels / 2);
		if (debug_) {
			ROS_INFO("[MazeDetector::detectLines] row: %d, col: %d, VSEG verticalLowerLeftX: %d, verticalLowerLeftXY %d, verticalUpperRightX: %d, verticalUpperRightY: %d, midpoint: %d",
					 row,
					 col,
					 verticalLowerLeftX,
					 verticalLowerLeftY,
					 verticalUpperRightX,
					 verticalUpperRightY,
					 midpoint);
		}

		return;
	} else if ((lengthOfContiguousColumnsOfWhitePixels >= (expectedLineSize * 2)) &&
			   (lengthOfContiguousColumnsOfWhitePixels <= (grey_image.cols * 0.7)) &&
			   (startOfContiguousColumnsOfWhitePixels <= (grey_image.cols * 0.1))) {
		// Seems like a part of a left-turn stripe.
		horizontalLowerLeftX = startOfContiguousColumnsOfWhitePixels;
		horizontalLowerLeftY = row;
		horizontalUpperRightX = col;
		horizontalUpperRightY = row;

		verticalLowerLeftY = -1;
		verticalUpperRightY = -1;
		verticalLowerLeftX = -1;
		verticalUpperRightX = -1;
		midpoint = startOfContiguousColumnsOfWhitePixels + (lengthOfContiguousColumnsOfWhitePixels / 2);
		if (debug_) {
			ROS_INFO("[MazeDetector::detectLines] row: %d, col: %d, LEFT horizontalLowerLeftX: %d, horizontalLowerLeftY: %d, horizontalUpperRightX: %d, verticalUpperRightY: %d, midpoint: %d",
					 row,
					 col,
					 horizontalLowerLeftX,
					 horizontalLowerLeftY,
					 horizontalUpperRightX,
					 horizontalUpperRightY,
					 midpoint);
		}

		return;
	} else if ((lengthOfContiguousColumnsOfWhitePixels >= (expectedLineSize * 2)) &&
			   (lengthOfContiguousColumnsOfWhitePixels <= (grey_image.cols * 0.7)) &&
			   (startOfContiguousColumnsOfWhitePixels > (grey_image.cols * 0.1))) {
		// Seems like a part of a right-turn stripe.
		horizontalLowerLeftX = startOfContiguousColumnsOfWhitePixels;
		horizontalLowerLeftY = row;
		horizontalUpperRightX = col;
		horizontalUpperRightY = row;

		verticalLowerLeftY = -1;
		verticalUpperRightY = -1;
		verticalLowerLeftX = -1;
		verticalUpperRightX = -1;
		midpoint = startOfContiguousColumnsOfWhitePixels + (lengthOfContiguousColumnsOfWhitePixels / 2);
		if (debug_) {
			ROS_INFO("[MazeDetector::detectLines] row: %d, col: %d, RIGHT horizontalLowerLeftX: %d, horizontalLowerLeftY: %d, horizontalUpperRightX: %d, verticalUpperRightY: %d, midpoint: %d",
					 row,
					 col,
					 horizontalLowerLeftX,
					 horizontalLowerLeftY,
					 horizontalUpperRightX,
					 horizontalUpperRightY,
					 midpoint);
		}

		return;
	} else if ((lengthOfContiguousColumnsOfWhitePixels >= (expectedLineSize * 4)) &&
			   (startOfContiguousColumnsOfWhitePixels <= (grey_image.rows * 0.1))) {
		// Seems like a part of a left-right-turn stripe.
		horizontalLowerLeftX = startOfContiguousColumnsOfWhitePixels;
		horizontalLowerLeftY = row;
		horizontalUpperRightX = col;
		horizontalUpperRightY = row;

		verticalLowerLeftY = -1;
		verticalUpperRightY = -1;
		verticalLowerLeftX = -1;
		verticalUpperRightX = -1;
		midpoint = startOfContiguousColumnsOfWhitePixels + (lengthOfContiguousColumnsOfWhitePixels / 2);
		if (debug_) {
			ROS_INFO("[MazeDetector::detectLines] row: %d, col: %d, LEFT-RIGHT horizontalLowerLeftX: %d, horizontalLowerLeftY: %d, horizontalUpperRightX: %d, verticalUpperRightY: %d, midpoint: %d",
					 row,
					 col,
					 horizontalLowerLeftX,
					 horizontalLowerLeftY,
					 horizontalUpperRightX,
					 horizontalUpperRightY,
					 midpoint);
		}

		return;
	} else {
		// Not a valid line.
		horizontalLowerLeftX = -1;
		horizontalLowerLeftY = -1;
		horizontalUpperRightX = -1;
		horizontalUpperRightY = -1;
		verticalLowerLeftY = -1;
		verticalUpperRightY = -1;
		verticalLowerLeftX = -1;
		verticalUpperRightX = -1;
		if (debug_) {
			ROS_INFO("[MazeDetector::detectLines] row: %d, col: %d, NOT LINE, reset startOfContiguousColumnsOfWhitePixels",
					 row,
					 col);
		}

		return;
	}
}

void MazeDetector::kmeansImage() {
	struct timespec start;
	int height = originalImage_.rows;
	int width = originalImage_.cols;
    Mat labels;
    int cluster_number = 4; // 10; // This is arbitrary. I increased it until I got the results I wanted.
    TermCriteria criteria {TermCriteria::COUNT, 100, 1};

	clock_gettime(CLOCK_MONOTONIC, &start);
    Mat centers;
    kmeans(reshaped_image32f, cluster_number, labels, criteria, 1, KMEANS_RANDOM_CENTERS, centers);
	// ProccTimePrint(start , "kmeans");

	// Build a histogram of how many pixels occur for each label.
	Mat labelHistogram(centers.rows, 1, CV_32S, Scalar(0));
    MatConstIterator_<int> labelFirst = labels.begin<int>();
    MatConstIterator_<int> labelLast = labels.end<int>();
	while (labelFirst != labelLast) {
		labelHistogram.at<int>(*labelFirst, 0) += 1;
		labelFirst++;
	}

	// Sort the histogram, descenting.
	Mat histSortIdx;
	sortIdx(labelHistogram, histSortIdx, SORT_EVERY_COLUMN + SORT_DESCENDING);
	// cout << "labelHistogram..." << endl << labelHistogram << endl;
	// ProccTimePrint(start , "sort histogram");

	// // Print out the sort index of each cluster.
	// cout << "histSortIdx..." << endl << histSortIdx << endl;

 //    // Print out the number of label rows (i.e., number of pixels) and label columns (there is only one).
 //    cout << "labels: " << labels.rows << " " << labels.cols << endl;

 //    // Print out the number of cluster rows (64, a.k.a centers) and cluster columns (3--r, g, b).
 //    cout << "centers: " << centers.rows << " " << centers.cols << endl;
    assert(labels.type() == CV_32SC1);
    assert(centers.type() == CV_32FC1);
     
    rgb_image = Mat(height, width, CV_8UC3);
    MatIterator_<Vec3b> rgb_first = rgb_image.begin<Vec3b>();
    MatIterator_<Vec3b> rgb_last = rgb_image.end<Vec3b>();
    MatConstIterator_<int> label_first = labels.begin<int>();
     
    Mat centers_u8;
    centers.convertTo(centers_u8, CV_8UC1, 255.0); // Convert the original centers to an array of clusters, scaled from [0..1) => [0..255].
    Mat centers_u8c3 = centers_u8.reshape(3);	// Convert to 3 columns (R, G, B).
 
    // Find the center with the min intensity.
    int minIntensity = 99999;
    int minIntensityCenter = -1;
    for (int center = 0; center < centers.rows; center++) {
        Vec3b vec3b =  centers_u8c3.ptr<Vec3b>(center)[0];
        int intensity = 0.114 * vec3b[0] + 0.587 * vec3b[1] + 0.299 * vec3b[2];
        if (intensity < minIntensity) {
            minIntensity = intensity;
            minIntensityCenter = center;
        }
    }

    // cout << "minIntensityCenter: " << minIntensityCenter << ", minIntensity: " << minIntensity << endl;

    // Blacken out everything that is not of interest.
    while ( rgb_first != rgb_last ) {
    		int cluster = *label_first;
            Vec3b& rgb = centers_u8c3.ptr<Vec3b>(cluster)[0];
            bool makeBlack = false;

            // if (cluster == histSortIdx.at<int>(0)) {
            // 	rgb = Vec3b(0, 0, 0);
            // } else if (cluster == histSortIdx.at<int>(1)) {
            // 	rgb = Vec3b(0, 0, 0);
            // } else if (cluster == histSortIdx.at<int>(2)) {
            // 	rgb = Vec3b(0, 0, 0);
            // }

            if (cluster != minIntensityCenter) rgb = Vec3b(0, 0, 0);
            else rgb = Vec3b(255, 255, 255);

            *rgb_first = rgb;
            ++rgb_first;
            ++label_first;
    }

	// Remove small objects from the forground.
	erode(rgb_image, rgb_image, getStructuringElement(MORPH_ELLIPSE, morphSize));
	dilate(rgb_image, rgb_image, getStructuringElement(MORPH_ELLIPSE, morphSize));
	cvtColor(rgb_image, grey_image, CV_BGR2GRAY);

  //   // Now add an annotation to the resulting picture. Add colored boxes to the right edge
  //   // of the picture, and print the cluster number and and histogram count next to the box.
  //   rectangle(rgb_image, Point(width - 25, 0), Point(width, (centers.rows * 8) + 12), Scalar::all(255), CV_FILLED);
  //   for (int cluster = 0; cluster < centers.rows; cluster++) {
		// Vec3b vec3b =  centers_u8c3.ptr<Vec3b>(cluster)[0];
  //       int intensity = 0.114 * vec3b[0] + 0.587 * vec3b[1] + 0.299 * vec3b[2];
		// Scalar color(vec3b);
		// // cout << "cluster: " << cluster
		// // 	 << ", int: " << intensity
		// // 	 << ", centers.rows: " << centers.rows
		// // 	 << endl;
		// rectangle(rgb_image, Point(width - 20, cluster * 8), Point(width, (cluster * 8) + 8), color, CV_FILLED);
		// char str[128];
		// sprintf(str, "%d %d", cluster, labelHistogram.at<int>(cluster, 0));
		// putText(rgb_image, str, Point(width - 60, (cluster * 8) + 7), FONT_HERSHEY_PLAIN, 0.5, Scalar::all(0), 1, 8);
  //   }
}

void MazeDetector::scaleOriginalImage(double scaleFactor) {
	if (1 || !originalImage_.empty()) {
		// Size scaleSize = originalImage_.size();
		// scaleSize.height *= scaleFactor;
		// scaleSize.width *= scaleFactor;
		// resize(originalImage_, originalImage_, scaleSize);

	    //resize(originalImage_, originalImage_, Size(), scaleFactor, scaleFactor, INTER_AREA);

		// ROS_INFO("originalImage_.cols: %d, originalImage_.rows: %d", originalImage_.cols, originalImage_.rows);
	    Mat reshaped_image = originalImage_.reshape(1, originalImage_.cols * originalImage_.rows);
	    // ROS_INFO("reshaped_image.cols: %d, reshaped_image.rows: %d", reshaped_image.cols, reshaped_image.rows);
	    assert(reshaped_image.type() == CV_8UC1);

	    reshaped_image.convertTo(reshaped_image32f, CV_32FC1, 1.0 / 255.0);
	    assert(reshaped_image32f.type() == CV_32FC1);
	}
}

MazeDetector::CURVE_FIT MazeDetector::linearCurveFit(std::vector<TLINE_SEGMENT> lineSegments) {
    float sumx = 0;
    float sumx2 = 0;
    float sumy = 0;
    float sumxy = 0;
    int n = lineSegments.size();
    for (int i = 0; i < n; i++) {
    	TLINE_SEGMENT lineSegment = lineSegments[i];
    	int x;
    	int y;
    	if (lineSegment.dir == H) {
    		y = lineSegment.y + (lineSegment.length / 2);
    		x = lineSegment.x;
    	} else {
    		y = lineSegment.x - (lineSegment.length / 2);
    		x = lineSegment.y;
    	}

        sumx = sumx + x;
        sumx2 = sumx2 + x * x;
        sumy = sumy + y;
        sumxy = sumxy + x * y;
	//        if (debug_LinearCurveFit) { cout << x << "\t" << y << endl; }
    }

    float t1 = sumx2 * sumy;
    float t2 = sumx * sumxy;
    float num = t1 - t2;
    float t3 = n * sumx2;
    float t4 = sumx * sumx;
    float denom = t3 - t4;
    float a = num / denom;
    float b = ((n * sumxy) - (sumx * sumy)) / ((n * sumx2) - (sumx * sumx));
    if (debug_LinearCurveFit) {
        cout << "n: " << n 
        	 << ", sumx: " << sumx
        	 << ", sumx2: " << sumx2
        	 << ", sumy: " << sumy
        	 << ", sumxy: " << sumxy
        	 << ", t1: " << t1 
        	 << ", t2: " << t2
        	 << ", num: " << num 
        	 << ", t3: " << t3 
        	 << ", t4: " << t4 
        	 << ", denom: " << denom 
        	 << ", a: " << a 
        	 << ", b: " << b << endl;
    }

    return CURVE_FIT(a, b);
}

std::string MazeDetector::TLINE_SEGMENT::toString() const {
    std::stringstream ss;
    ss << "{"
         << "x:" << x
         << ", y:" << y
         << ", l:"  << length
         << ", d: " << dir
         << "}";
    return ss.str();
}