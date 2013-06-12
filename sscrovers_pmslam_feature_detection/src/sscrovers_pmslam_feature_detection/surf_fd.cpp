/*
 * surf_fd.cpp
 *
 *  Created on: 9 Jul 2012
 *      Author: sscrovers
 */

#include "surf_fd.h"

SurfFD::SurfFD()
{
  // TODO Auto-generated constructor stub
  keypoints_ = 0, descriptors_ = 0;
}

SurfFD::~SurfFD()
{
  // TODO Auto-generated destructor stub
}

void SurfFD::extractKeypoints(cv::Mat* m)
{
  //memcpy( output_image_, input_image_, sizeof(input_image_) );//Replace by opencv copying

  //CvSURFParams params = cvSURFParams(500, 1);
  //CvMemStorage* storage = cvCreateMemStorage(0);
  //(*outputEK).extractKeypoints();

  CvSURFParams params = cvSURFParams(500, 0);
  CvMemStorage* storage = cvCreateMemStorage(0);

  IplImage _img(*m);
  cvExtractSURF(&_img, 0, &keypoints_, &descriptors_, storage, params);
}
