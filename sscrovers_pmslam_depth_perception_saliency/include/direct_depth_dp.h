/*
 * surf_fd.h
 *
 *  Created on: 9 Jul 2012
 *      Author: sscrovers
 */

#ifndef DirectDepthDP_H_
#define DirectDepthDP_H_

//Open CV
#include <cv.h>

//pmslam structures
#include "RoverState.h"
#include "SURFPoint.h"

//feature database handling
#include "features_db.h"

using std::string;

typedef FeaturesDB<SURFPoint, RoverState, CvSURFPoint> FeaturesDB_t;

class DirectDepthDP
{
public:
  DirectDepthDP();
  virtual ~DirectDepthDP();

  void directDepth(vector<SURFPoint>* SURFDatabase, vector<int>& ptpairsin, vector<int>& ptpairsout, RoverState *state,
                   vector<CvPoint3D64f> &points3D, double rng, int px, int py, double VFOV, double HFOV, double pan,
                   double tilt, double CamH);

  //! Class member containing a sequence of image keypoints.
  CvSeq *keypoints_ptr_;

  //! Class member containing a sequence of keypoint descriptors.
  CvSeq *descriptors_ptr_; //useless!!

private:
  void transformAngles(float yaw, float pitch, float roll, double *alpha, double *beta, double *gamma);
  double absoluteDistance(double x, double y, double z);
};

#endif /* DirectDepthDP_H_ */
