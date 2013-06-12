/*
 * AFeatures.h
 *
 *  Created on: Sep 6, 2012
 *      Author: weclewski
 */

#ifndef AFeature_H_
#define AFeature_H_

#include<vector>
#include<string>
#include<stdint.h>

namespace pmslam
{

class AFeature
{
public:
  AFeature();
  virtual ~AFeature();

  virtual std::string descriptionHeaderToStream() =0;
  virtual std::string toStream() = 0;
  virtual double match(const AFeature* f_ptr) const = 0;
  virtual void match(const std::vector<AFeature>* f_vec_ptr, std::vector<int>* ptpairs) const = 0;

  virtual void serialize(uint8_t* buff_ptr) = 0;
  virtual void serialize(std::vector<uint8_t>* buff_vec_ptr) = 0;

  virtual void deserialize(uint8_t* buff_ptr, size_t n) = 0;
  virtual void deserialize(std::vector<uint8_t>* buff_vec_ptr) = 0;
};

} /* namespace pmslam */
#endif /* AFeature_H_ */
