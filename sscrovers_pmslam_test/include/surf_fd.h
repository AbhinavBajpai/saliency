/*
 * SurfFD.h
 *
 *  Created on: 9 Jul 2012
 *      Author: sscrovers
 */

#ifndef SURFFD_H_
#define SURFFD_H_

#include <cv.h>

class SurfFD {
public:
	SurfFD();
	virtual ~SurfFD();
	void extractKeypoints(cv::Mat* m);


	//	cv::Mat img;		// new image class (openCV 2)

	//! Class member containing a sequence of image keypoints.
	CvSeq *keypoints_;

	//! Class member containing a sequence of keypoint descriptors.
	CvSeq *descriptors_;
};

#endif /* SURFFD_H_ */
