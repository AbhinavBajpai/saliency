/*
 * surf_fd.h
 *
 *  Created on: 9 Jul 2012
 *      Author: sscrovers
 */

#ifndef TriangulateDP_H_
#define TriangulateDP_H_

#include <opencv2/imgproc/imgproc.hpp>	// new library
#include <opencv2/highgui/highgui.hpp>	// new library
//#include <opencv/cv.h>				// old library
//#include <opencv/highgui.h>			// old library
//#include <cv_bridge/cv_bridge.h>	// new library
//#include <cv_bridge/CvBridge.h>		// old library //* TODO change to new ver
//#include <image_transport/image_transport.h>

using namespace cv;
using std::string;

class TriangulateDP {
public:
	TriangulateDP();
	virtual ~TriangulateDP();
	
	void transformAngles(float yaw, float pitch, float roll, double *alpha, double *beta, double *gamma);
	double absoluteDistance(double x, double y, double z);
	void triangulate();

	//! Class member containing a sequence of image keypoints.
	CvSeq *keypoints_;

	//! Class member containing a sequence of keypoint descriptors.
	CvSeq *descriptors_;
};

#endif /* TriangulateDP_H_ */
