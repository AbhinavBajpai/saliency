#ifndef InformationFilterFtr_H_
#define InformationFilterFtr_H_

#include "EIF.h"
#include "RoverState.h"//to remove?
#include "Map.h"//to remove
#include "sscrovers_pmslam_common/SPoint.h"
#include <vector>
#include "ros/ros.h"

using std::string;

class InformationFilterFtr
{
public:
  InformationFilterFtr(ros::NodeHandle * nh);
  virtual ~InformationFilterFtr();

  //! size of the information matrix Lambda
  int lambda_size_;
  //! odometry error
  double odom_err_;
  //! size of the rover state vector
  int rover_state_vec_size_;
  //! size of the measurement state vector
  int measuered_state_vec_size_;

  Map* map3d_ptr_;

  struct PMSLAM_DATA_OLD_t
  {
    struct
    {
      vector<int> ID;
      struct
      {
        vector<double> x;
        vector<double> y;
        vector<double> z;
      } positions;

    } MapOut;
    CvPoint3D32f TrajectoryOut;
  } PMSLAM_Data_msg;

  void Init(RoverState* state);
  void update(int step, RoverState s, std::vector<sscrovers_pmslam_common::SPoint> sVec);
  void CreateMap(Mat *u, vector<int> *ptpairsCMin, vector<CvPoint3D64f> *pointsCMin);

private:
  RoverState curr_pose_ptr_;
  int step_ptr_;

  EIF* slam_filter_ptr_;

  std::vector<sscrovers_pmslam_common::SPoint> db_ptr_;
  ros::NodeHandle n;
  std::vector <int> idVec;

};

#endif /* InformationFilterFtr_H_ */
