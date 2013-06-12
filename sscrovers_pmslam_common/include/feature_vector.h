/*
 * surf_vector.h
 *
 *  Created on: Sep 6, 2012
 *      Author: weclewski
 */

#ifndef SURF_VECTOR_H_
#define SURF_VECTOR_H_

#include "feature.h"

namespace pmslam
{

class FeatureVector : public AFeature
{

public:
  FeatureVector();
  virtual ~FeatureVector();

  std::vector<Feature> storage_;

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

#endif /* SURF_VECTOR_H_ */
