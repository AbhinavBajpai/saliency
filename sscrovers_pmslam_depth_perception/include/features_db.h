/*
 * features_db.h
 *
 *  Created on: 25 Jul 2012
 *      Author: sscrovers
 */

#ifndef FEATURES_DB_H_
#define FEATURES_DB_H_

#include <vector>

template <class INT_PT, class POSE, class EXT_PT>
class FeaturesDB {
public:
	FeaturesDB();
	virtual ~FeaturesDB();

	//! surf features db temp object
	std::vector<INT_PT > *storage_; //TODO change it to methods for use external  db

	//! initialisation
	void dbInit();

	//! add new record to db
	void dbAdd(int db_key, EXT_PT* surf_keypoint, float keypoint_descriptors[64], POSE* observation_point, float point_ground_truth[3], int number_of_observations, int related_landmark_index);

	//!
	void dbMatch();
	void dbGet();
	void dbMod(int idx, EXT_PT* surf_keypoint);
	void dbRelease();
	//void icreaseObs();
};

template<class INT_PT, class POSE, class EXT_PT>
FeaturesDB<INT_PT, POSE, EXT_PT>::FeaturesDB()
{
	dbInit();
}

template<class INT_PT, class POSE, class EXT_PT>
FeaturesDB<INT_PT, POSE, EXT_PT>::~FeaturesDB()
{
	dbRelease();
}

template<class INT_PT, class POSE, class EXT_PT>
void FeaturesDB<INT_PT, POSE, EXT_PT>::dbInit()
{
	storage_ 	= new std::vector<INT_PT>;//temp
}

template<class INT_PT, class POSE, class EXT_PT>
void FeaturesDB<INT_PT, POSE, EXT_PT>::dbAdd(int db_key, EXT_PT* surf_keypoint, float keypoint_descriptors[64], POSE* observation_point, float point_ground_truth[3], int number_of_observations, int related_landmark_index)
{
	(*storage_).push_back(INT_PT(db_key, *surf_keypoint, keypoint_descriptors, *observation_point, point_ground_truth, number_of_observations, related_landmark_index));
}

template<class INT_PT, class POSE, class EXT_PT>
void FeaturesDB<INT_PT, POSE, EXT_PT>::dbMatch()
{

}

template<class INT_PT, class POSE, class EXT_PT>
void FeaturesDB<INT_PT, POSE, EXT_PT>::dbGet()
{

}

template<class INT_PT, class POSE, class EXT_PT>
void FeaturesDB<INT_PT, POSE, EXT_PT>::dbMod(int idx, EXT_PT* surf_keypoint)
{

}

template<class INT_PT, class POSE, class EXT_PT>
void FeaturesDB<INT_PT, POSE, EXT_PT>::dbRelease()
{
	storage_->clear();
	delete storage_;
}

#endif /* FEATURES_DB_H_ */
