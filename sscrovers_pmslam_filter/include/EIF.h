/** This file defines the EIF class and related functions.

 *	@date 11/11/2010
 *	@author Karin Shala
 *
 *   @detail The information filter implementation is based on:
 *			M. R. Walter, R. M. Eustice, and J. J. Leonard, �Exactly sparse
 *			extended information filters for feature-based SLAM,�
 *			International Journal of Robotics Research, vol. 26, no. 4, pp. 335�359, 2007.
 *
 */

#ifndef EIF_H
#define EIF_H

//#include "opencv/cv.h"
//#include <opencv2/core/core.hpp> //PW
#include <cv.h> //PW
#include <cmath>
#include <math.h>
#include <iostream>

//#include "ros/ros.h" //PW

using namespace cv;
//using namespace std;

/** @class EIF.h "src/EIF.h"
 *  @brief This is the EIF class.
 *
 *  @detail The EIF class contains data structures and functions for the
 *          Extended Information Filter.
 */
class EIF
{

public:

  /** Default Constructor
   */
  EIF();

  /** Constructor taking arguments for initialisation.
   * @param[in] matsz size of the information matrix Lambda
   * @param[in] oderr odometry error
   * @param[in] x initial x position
   * @param[in] y initial y position
   * @param[in] z initial z position
   * @param[in] yaw initial yaw position
   * @param[in] s size of the rover state vector
   * @param[in] ms size of the measurement state vector
   */
  EIF(int matsz, double oderr, double x, double y, double z, double yaw, int s, int ms);

  /** Destructor
   */
  ~EIF();

  /** Class member for information matrix.
   */
  Mat *lambda;

  /** Class member for information vector.
   */
  Mat *eta;

  /** Class member for mean state vector.
   */
  Mat *mu;

  /** Class member for motion covariance matrix.
   */
  Mat *Q;

  /** Class member for measurement covariance matrix.
   */
  Mat *R;

  /** Class member for index to which lambda and eta are filled.
   */
  int idx;

  /** Class member for rover state size.
   */
  int *S;

  /** Class member for measurement state size (not in use).
   */
  int *MS;

  /** Member function for intitialisation (same as constructor taking arguments).
   *
   * @param[in] matsz size of the information matrix lambda
   * @param[in] oderr odometry error
   * @param[in] x initial x position
   * @param[in] y initial y position
   * @param[in] z initial z position
   * @param[in] yaw initial yaw position
   * @param[in] s size of the rover state vector
   * @param[in] ms size of the measurement state vector
   * @return void
   *
   * @detail This function initialises the parameters of the information filter.
   *
   */
  void init(int matsz, double oderr, double x, double y, double z, double yaw, int s, int ms);

  /** Member function for filter prediction.
   *
   * @param[in] u control vector
   * @return void
   *
   * @detail This function implements the information filter prediction step.
   *
   */
  void predict(Mat u);

  /** Member function for filter augmentation.
   *
   * @param[in] vn vector containing measurements of new landmarks
   * @param[in] vr vector containing measurement covariance for each measurement in vn
   * @return void
   *
   * @detail This function implements the information filter augmentation step.
   *
   */
  void augment(vector<CvPoint3D64f> vn, vector<double> vr);

  /** Class member function for filter update.
   *
   * @param[in] vf vector containing measurements of re-observed landmarks
   * @param[in] vr vector containing measurement covariance for each measurement in vf
   * @return void
   *
   * @detail This function implements the information filter update step.
   *
   */
  void update(vector<CvPoint3D64f> vf, vector<double> vr, vector<int> *id);

  /** Class member function for mean recovery.
   *
   * @return void
   *
   * @detail This function recovers the mean state vector from the information filter.
   *
   */
  void recoverMean();

  /** Class member function for exactly sparse extended information filter marginalisation (not implemented).
   */
  void marginalise(vector<int> *id);

  /** Class member function for exactly sparse extended information filter relocalisation (not implemented).
   */
  void relocalise(vector<CvPoint3D64f> vb, vector<int> *id);

};

#endif
