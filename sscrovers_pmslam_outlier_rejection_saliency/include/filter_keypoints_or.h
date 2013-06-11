#ifndef FilterKeypointsOR_H_
#define FilterKeypointsOR_H_

#include <opencv2/imgproc/imgproc.hpp>	// new library
#include <opencv2/highgui/highgui.hpp>	// new library
#include "surf.h"

#include "features_db.h"

typedef FeaturesDB<SURFPoint, RoverState, CvSURFPoint> FeaturesDB_t;

class FilterKeypointsOR
{
public:
  FilterKeypointsOR(int* step, RoverState* curr_pos, FeaturesDB_t* db);
  virtual ~FilterKeypointsOR();

  //! pmslam parameters
  int mode_;
  int px_, py_;
  bool disp_f_;
  bool save_f_;
  int lm_track_; //94
  int scale_;

  void filterKeypoints();

  //! Class member containing a sequence of image keypoints.
  CvSeq *keypoints_ptr_;

  //! Class member containing a sequence of keypoint descriptors.
  CvSeq *descriptors_ptr_;

  //! Point pairs
  vector<int> pt_pairs_;

private:
  //! current pose
  RoverState* curr_pose_ptr_;

  //! current step
  int* step_;

  //! features db
  FeaturesDB_t* surf_db_ptr_;

  void matchDB(vector<SURFPoint>* SURFDatabase, vector<int>& ptpairs, int px, int py); //TODO to move out
  void addToDatabase(vector<SURFPoint> *SURFDatabase, RoverState *state, int px, int py); //TODO to move out
  /*!
   * \brief key points filtration based on stored data base
   *
   * PMSLAM CHANGES:
   * - Image class restructured to needed functions
   * - Simple rename (see: begin of source)
   * 			ptpairsFKout = &pt_pairs_;
   * 			SURFDatabase = features_db_;
   *			image = this;
   */

  void rejectOutliers(vector<SURFPoint>* SURFDatabase, vector<int>& ptpairs); //TODO to move out??

  void save(bool showkp, vector<SURFPoint> *SURFDatabase, vector<int> *ptpairsCurrent, int ImgScale, int lmTracking,
            int mode);
  void display(bool showkp, vector<SURFPoint> *SURFDatabase, vector<int> *ptpairsCurrent, int ImgScale, int lmTracking);
};

#endif /* FilterKeypointsOR_H_ */
