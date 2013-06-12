/*
 * surf_feture.cpp
 *
 *  Created on: Sep 6, 2012
 *      Author: weclewski
 */

#include "feature.h"

namespace pmslam
{

const size_t Feature::__byte_size = sizeof(Feature);
const size_t Feature::__descriptor_size = 64;
const size_t Feature::__gtruth_size = 3;
const size_t Feature::__pxLocation_size = 2;
double Feature::compare_dist_ = 1e6;

Feature::Feature()
{
  // TODO Auto-generated constructor stub
  flag_ = 0;
  id_ = 0;
  index_ = 0;
  n_ = 0;
  step_ = 0;
}

Feature::~Feature()
{
  // TODO Auto-generated destructor stub
}

std::string Feature::descriptionHeaderToStream()
{
  std::stringstream ss;
  ss << "#\t index\n" << "pt.pt.x\t pt.pt.y\t lap\t size\t dir\t hess\n"
      << "state.x\t state.y\t state.z\t state.yaw\t state.pitch\t state.roll\n" << "pxLocation[0]\t pxLocation[1]\n"
      << "gtruth[0]\t gtruth[1]\t gtruth[2]\n" << "n\t id\t step\t flag\n" << "desc[64]" << std::endl << std::endl;
  return ss.str();
}

std::string Feature::toStream()
{
  std::stringstream ss;

  ss << index_ << std::endl << pt_.pt.x << "\t" << pt_.pt.y << "\t" << pt_.laplacian << "\t" << pt_.size << "\t"
      << pt_.dir << "\t" << pt_.hessian << std::endl << state_.x << "\t" << state_.y << "\t" << state_.z << "\t"
      << state_.yaw << "\t" << state_.pitch << "\t" << state_.roll << std::endl << pxLocation_[0] << "\t"
      << pxLocation_[1] << std::endl << gtruth_[0] << "\t" << gtruth_[1] << "\t" << gtruth_[2] << std::endl << "\t"
      << n_ << "\t" << id_ << "\t" << step_ << "\t" << flag_ << std::endl;

  for (unsigned int j = 0; j < __descriptor_size; j++)
  {
    ss << descriptor_[j] << "\t";
  }
  ss << std::endl << std::endl;

  return ss.str();
}

double compareSURFDescriptors(const float* d1, const float *d2, double best, int length)
{
  double total_cost = 0;
  assert( length % 4 == 0);
  for (int i = 0; i < length; i += 4)
  {
    double t0 = d1[i] - d2[i];
    double t1 = d1[i + 1] - d2[i + 1];
    double t2 = d1[i + 2] - d2[i + 2];
    double t3 = d1[i + 3] - d2[i + 3];
    total_cost += t0 * t0 + t1 * t1 + t2 * t2 + t3 * t3;
    if (total_cost > best)
      break;
  }
  return total_cost;
}

double Feature::match(const AFeature* f_ptr) const
{
  if (pt_.laplacian != static_cast<const Feature*>(f_ptr)->pt_.laplacian)
    return -1;
  return compareSURFDescriptors(descriptor_, static_cast<const Feature*>(f_ptr)->descriptor_, compare_dist_,
                                __descriptor_size);
}

void Feature::match(const std::vector<AFeature>* f_vec_ptr, std::vector<int>* ptpairs) const
{
  //not resolved yet
  /*
   if (pt_.laplacian != static_cast<const Feature*>(f_ptr)->pt_.laplacian)
   return -1;
   return compareSURFDescriptors(descriptor_, static_cast<const Feature*>(f_ptr)->descriptor_, compare_dist_,
   __descriptor_size);
   */
}

void Feature::serialize(uint8_t* buff_ptr)
{
  if (!buff_ptr)
    buff_ptr = new uint8_t[__byte_size];

  //until feature class contents only regular types (generic types and static arrays), it is possible to copy it in simple way. Otherwise there is need to serialize field by field.
  std::copy((uint8_t*)this, ((uint8_t*)this) + __byte_size, buff_ptr);
}

void Feature::serialize(std::vector<uint8_t>* buff_vec_ptr)
{
  buff_vec_ptr->resize(__byte_size);
  this->serialize(buff_vec_ptr->data());
}

void Feature::deserialize(uint8_t* buff_ptr, size_t n)
{
  if (n == __byte_size)
  {
    //until feature class contents only regular types (generic types and static arrays), it is possible to copy it in simple way. Otherwise there is need to serialize field by field.
    std::copy(buff_ptr, buff_ptr + __byte_size, (uint8_t*)this);
  }
  else
    throw;

}

void Feature::deserialize(std::vector<uint8_t>* buff_vec_ptr)
{
  if (buff_vec_ptr->size() == __byte_size)
  {
    this->deserialize(buff_vec_ptr->data(), buff_vec_ptr->size());
  }
  else
    throw;
}

}/* namespace pmslam */
