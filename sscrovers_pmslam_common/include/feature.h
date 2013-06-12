/*
 * surf_feture.h
 *
 *  Created on: Sep 6, 2012
 *      Author: weclewski
 */

#ifndef SURF_FEATURE_H_
#define SURF_FEATURE_H_

//STL
#include <string>
#include <exception>

//OpenCV
#include <cv.h>

#include "afeature.h"
#include "rover_state.h"

using namespace cv;

namespace pmslam
{

class Feature : public AFeature
{
public:
  Feature();
  virtual ~Feature();

  //! byte size of class
  static const size_t __byte_size;

  //! distance (error) for comparison features
  static double compare_dist_;

  //! Class member for unique index of the SURF point.
  int index_;

  //! Class member for OpenCV SURF keypoint.
  CvSURFPoint pt_;

  //! Class member for OpenCV SURF descriptor
  static const size_t __descriptor_size;
  float descriptor_[64];

  //! Class member for the rover state the SURF point was observed at.
  RoverState state_;

  //! Class member for the location of the SURF point in the last observed image.
  static const size_t __pxLocation_size;
  int pxLocation_[2];

  //! Class member for the SURF point's ground truth in 3D world coordinates.
  static const size_t __gtruth_size;
  float gtruth_[3];

  //! Class member counting the number of times the SURF point is observed.
  int n_;

  //! Class member for index of corresponding landmark in the map (0 if no landmark is assigned).
  int id_;

  //! Class member indicating the last step at which the SURF point is observed.
  int step_;

  /** Class member indicating if (0) the SURF point was not observed in the previous iteration or
   *    (1) the SURF point was observed in the previous iteration or
   *    (2) the SURF point was not observed in the previous 10 iterations.
   */
  int flag_;

  std::string descriptionHeaderToStream();
  std::string toStream();

  double match(const AFeature* f_ptr) const;
  void match(const std::vector<AFeature>* f_vec_ptr, std::vector<int>* ptpairs) const;

  void serialize(uint8_t* buff_ptr);
  void serialize(std::vector<uint8_t>* buff_vec_ptr);

  void deserialize(uint8_t* buff_ptr, size_t n);
  void deserialize(std::vector<uint8_t>* buff_vec_ptr);
};

} /* namespace pmslam */

#endif /* SURF_FEATURE_H_ */
