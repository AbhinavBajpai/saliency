/*
 * direct_depth_dp.cpp
 *
 *  Created on: 9 Jul 2012
 *      Author: sscrovers
 */

#include "direct_depth_dp.h"

DirectDepthDP::DirectDepthDP()
{
  // TODO Auto-generated constructor stub
}

DirectDepthDP::~DirectDepthDP()
{
  // TODO Auto-generated destructor stub
}

void DirectDepthDP::transformAngles(float yaw, float pitch, float roll, double *alpha, double *beta, double *gamma)
{
  const double PI = 3.141592;

  *alpha = roll * PI / 180;
  *beta = pitch * PI / 180;
  *gamma = yaw * PI / 180;
}

double DirectDepthDP::absoluteDistance(double x, double y, double z)
{
  double r;
  r = sqrt(x * x + y * y + z * z);
  return r;
}

void DirectDepthDP::directDepth(std::vector<sscrovers_pmslam_common::SPoint>* SDatabase, vector<int>& ptpairsin, vector<int>& ptpairsout,
                                RoverState *state, vector<CvPoint3D64f> &points3D, double rng, int px, int py,
                                double VFOV, double HFOV, double pan, double tilt, double CamH)
{


  const double PI = 3.141592;
  VFOV = VFOV * PI / 180;
  HFOV = HFOV * PI / 180;

  CvPoint3D64f pt3D;
  vector<int> pts;
  pts.clear();
  points3D.clear();

  double d, u, v, x, y, alpha, beta, gamma; //, z, alpha=roll, beta=pitch gamma=yaw (in radians)

  for (unsigned int j = 0; j < ptpairsin.size(); j ++)
  {
    //CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(keypoints, ptpairsin[j]);
    sscrovers_pmslam_common::SPoint ss = SDatabase->at(ptpairsin[j]);

    u = ss.x / px;
    v = ss.y / py;

    // insure the sign of the pitch angle is correct
    y = (state->z + CamH) / tan(((state->pitch + tilt) * PI / 180) + atan((2 * v - 1) * tan(VFOV / 2)));

    //              x = y * (2*u - 1)*tan(HFOV/2);
    x = y * tan(((state->yaw + pan) * PI / 180) + atan((2 * u - 1) * tan(HFOV / 2))); // yaw does not change local coordinates

    //CvMat Pt, *transMat, PR, PT, *RR, *TT;

    // alpha=roll, beta=pitch gamma=yaw (in radians)
    transformAngles(state->yaw, state->pitch, state->roll, &alpha, &beta, &gamma);

    Mat point(3, 1, CV_64FC1);

    point.at<double>(0, 0) = x;
    point.at<double>(1, 0) = y;
    point.at<double>(2, 0) = 0;

    //// translation matrix
    //Mat T = Mat::zeros(3, 1, CV_64F);
    //T.at<double>(0,0) = state->x;
    //T.at<double>(1,0) = state->y;
    //T.at<double>(2,0) = state->z;

    // Estimation of change in yaw angle

    // rotation matrix
    Mat R = Mat::zeros(3, 3, CV_64F);
    R.at<double>(0, 0) = cos(alpha) * cos(gamma) - sin(alpha) * sin(beta) * sin(gamma);
    R.at<double>(1, 0) = -cos(beta) * sin(gamma);
    R.at<double>(2, 0) = cos(alpha) * sin(beta) * sin(gamma) - cos(gamma) * sin(alpha);
    R.at<double>(0, 1) = cos(alpha) * sin(gamma) - cos(gamma) * sin(alpha) * sin(beta);
    R.at<double>(1, 1) = cos(beta) * cos(gamma);
    R.at<double>(2, 1) = -sin(alpha) * sin(gamma) - cos(alpha) * cos(gamma) * sin(beta);
    R.at<double>(0, 2) = cos(beta) * sin(alpha);
    R.at<double>(1, 2) = sin(beta);
    R.at<double>(2, 2) = cos(alpha) * cos(beta);

    // rotation from camera into world coordinate system
    Mat pointW = R.inv() * point;

    pt3D.x = pointW.at<double>(0, 0);
    pt3D.y = pointW.at<double>(1, 0);
    pt3D.z = 0; // not used

    // Removing point pairs that are beyond the range set in the PMSLAM.ini setup file
    d = absoluteDistance(pt3D.x, pt3D.y, pt3D.z);
    if (d < rng)
    {
      points3D.push_back(pt3D);
      pts.push_back(ptpairsin[j]);
    }
  }
  ptpairsout = pts;
}
