/*
 * surf_vector.cpp
 *
 *  Created on: Sep 6, 2012
 *      Author: weclewski
 */

#include "feature_vector.h"

namespace pmslam
{
/*
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
 */
int naiveNearestNeighbor(const Feature* f, const vector<Feature>* vec)
{
  //int length = Feature::__descriptor_size;
  int neighbor = -1;
  double d, dist1 = 1e6, dist2 = 1e6;

  for (unsigned int i = 0; i < vec->size(); i++)
  {
    /*
     if (f->pt_.laplacian != (*vec)[i].pt_.laplacian)
     continue;
     d = compareSURFDescriptors(f->descriptor_, (*vec)[i].descriptor_, dist2, length);
     */
    //use above or match features by Feature method below - the same code :)
    d = f->match(&((*vec)[i]));

    if (d < dist1)
    {
      dist2 = dist1;
      dist1 = d;
      neighbor = i;
    }
    else if (d < dist2)
      dist2 = d;

  }

  if (dist1 < 0.6 * dist2)
    return neighbor;
  return -1;
}

void findPairs(const vector<Feature>* database, const vector<Feature>* vec, vector<int>* ptpairs)
{
  unsigned int i;
  ptpairs->clear();

  for (i = 0; i < database->size(); i++)
  {
    int nearest_neighbor = naiveNearestNeighbor(&((*database)[i]), vec);

    if (nearest_neighbor >= 0)
    {
      ptpairs->push_back(i); //TODO don't like it :P
      ptpairs->push_back(nearest_neighbor);
    }
  }
}

void flannFindPairs(const vector<Feature>* database, const vector<Feature>* vec, vector<int>* ptpairs)
{
  int length = Feature::__descriptor_size;

  ptpairs->clear();

  cv::Mat m_object(database->size(), length, CV_32F);
  cv::Mat m_image(vec->size(), length, CV_32F);

  // copy descriptors
  float* obj_ptr = m_object.ptr<float>(0);
  for (unsigned int i = 0; i < database->size(); i++)
  {
    std::copy((*database)[i].descriptor_, (*database)[i].descriptor_ + length * sizeof(float), obj_ptr);
    obj_ptr += length;
  }

  float* img_ptr = m_image.ptr<float>(0);
  for (unsigned int i = 0; i < database->size(); i++)
  {
    std::copy((*vec)[i].descriptor_, (*vec)[i].descriptor_ + length * sizeof(float), img_ptr);
    img_ptr += length;
  }

  // find nearest neighbors using FLANN
  cv::Mat m_indices(database->size(), 2, CV_32S);
  cv::Mat m_dists(database->size(), 2, CV_32F);
  cv::flann::Index flann_index(m_image, cv::flann::KDTreeIndexParams(4)); // using 4 randomized kdtrees
  flann_index.knnSearch(m_object, m_indices, m_dists, 2, cv::flann::SearchParams(64)); // maximum number of leafs checked

  int* indices_ptr = m_indices.ptr<int>(0);
  float* dists_ptr = m_dists.ptr<float>(0);
  for (int i = 0; i < m_indices.rows; ++i)
  {
    if (dists_ptr[2 * i] < 0.6 * dists_ptr[2 * i + 1])
    {
      ptpairs->push_back(i);
      ptpairs->push_back(indices_ptr[2 * i]);
    }
  }
}

FeatureVector::FeatureVector()
{
  // TODO Auto-generated constructor stub
  storage_.clear();
}

FeatureVector::~FeatureVector()
{
  // TODO Auto-generated destructor stub
}

std::string FeatureVector::descriptionHeaderToStream()
{
  std::stringstream ss;
  Feature* sf = new Feature();
  ss << "total: " << storage_.size() << std::endl << sf->descriptionHeaderToStream();
  delete sf;
  return ss.str();
}

std::string FeatureVector::toStream()
{
  std::stringstream ss;
  for (unsigned int i = 0; i < storage_.size(); i++)
  {
    ss << "#" << i << std::endl;
    ss << storage_[i].toStream() << std::endl;
  }
  return ss.str();
}

double FeatureVector::match(const AFeature* f_ptr) const
{
  //not resolved yet
  return -1;
}

void FeatureVector::match(const std::vector<AFeature>* f_vec_ptr, std::vector<int>* ptpairs) const
{
  findPairs(&storage_, (std::vector<Feature>*)f_vec_ptr, ptpairs);
  //flannFindPairs(&storage_, (std::vector<Feature>*)f_vec_ptr, ptpairs);
}

void FeatureVector::serialize(uint8_t* buff_ptr)
{
  size_t size = storage_.size() * Feature::__byte_size;
  if (!buff_ptr)
    buff_ptr = new uint8_t[size];
  //std::copy(storage_.data(), storage_.data() + size, buff_ptr);
  for (unsigned int i = 0; i < storage_.size(); i++)
    storage_[i].serialize(buff_ptr + i * Feature::__byte_size);
}

void FeatureVector::serialize(std::vector<uint8_t>* buff_vec_ptr)
{
  buff_vec_ptr->resize(storage_.size() * Feature::__byte_size);
  this->serialize(buff_vec_ptr->data());
}

void FeatureVector::deserialize(uint8_t* buff_ptr, size_t n)
{
  Feature f;
  if (n % Feature::__byte_size)
    throw;
  else
  {
    storage_.resize(n / Feature::__byte_size);
    for (unsigned int i = 0; i < storage_.size(); i++)
      storage_[i].deserialize(buff_ptr + i * Feature::__byte_size, Feature::__byte_size);
  }
}

void FeatureVector::deserialize(std::vector<uint8_t>* buff_vec_ptr)
{
  if (buff_vec_ptr->size() % Feature::__byte_size)
    throw;
  else
    this->deserialize(buff_vec_ptr->data(), buff_vec_ptr->size());
}

}/* namespace pmslam */
