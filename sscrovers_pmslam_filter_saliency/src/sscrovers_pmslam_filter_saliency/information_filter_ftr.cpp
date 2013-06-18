#include "information_filter_ftr.h"


InformationFilterFtr::InformationFilterFtr(ros::NodeHandle * nh) 
{
  slam_filter_ptr_ = new EIF();
  map3d_ptr_ = new Map();
  ros::param::set("filter",false);


}

InformationFilterFtr::~InformationFilterFtr()
{
  delete slam_filter_ptr_;
  delete map3d_ptr_;
}

void InformationFilterFtr::update(int step, RoverState s, std::vector<sscrovers_pmslam_common::SPoint> sVec)
{
	curr_pose_ptr_= s;
	step_ptr_ = step;
	db_ptr_ = sVec;
	idVec.resize(sVec.size());
	
}

void InformationFilterFtr::Init(RoverState* state)
{
  slam_filter_ptr_->init(lambda_size_, odom_err_, state->x, state->y, state->z, state->yaw, rover_state_vec_size_,
                         measuered_state_vec_size_);
}

void InformationFilterFtr::CreateMap(Mat *u, vector<int> *ptpairsCMin, vector<CvPoint3D64f> *pointsCMin)
{
  //re-mapping
  Map* Map3D = map3d_ptr_;
  typedef struct
  {
    vector<RoverState> tEST;
  } RoverTrajectory_t;
  RoverTrajectory_t* RoverTrajectory = new RoverTrajectory_t;
  RoverTrajectory->tEST.resize(step_ptr_ + 1);
  RoverTrajectory->tEST[step_ptr_] = curr_pose_ptr_;
  int* STEP = &step_ptr_;
  vector<sscrovers_pmslam_common::SPoint>* SDatabase = &db_ptr_;

  int id;
  vector<CvPoint3D64f> vn, vf;
  vector<double> vrn, vrf;
  vector<int> ids;
  int idx = slam_filter_ptr_->idx;

  PMSLAM_Data_msg.MapOut.ID.clear();
  PMSLAM_Data_msg.MapOut.positions.x.clear();
  PMSLAM_Data_msg.MapOut.positions.y.clear();
  PMSLAM_Data_msg.MapOut.positions.z.clear();
ROS_INFO("pp = %lu", ptpairsCMin->size());
  if (*STEP > 0)
  {
    slam_filter_ptr_->predict(*u);
    vn.clear();
    vf.clear();
    vrn.clear();
    vrf.clear();
    ids.clear();

    CvPoint3D64f temp;

    // Split landmarks into new (vn) and re-observed (vf) ones
    for (unsigned int j = 0; j < ptpairsCMin->size(); j ++)
    {

      ros::param::set("filter", true);	
      temp.x = (*pointsCMin)[j ].x;
      temp.y = (*pointsCMin)[j ].y;
      temp.z = (*pointsCMin)[j ].z;

      if ((*SDatabase)[(*ptpairsCMin)[j]].id == 0) // if new landmark
      {
//ROS_INFO("New Landmark");
//lnadmark added
	

        vn.push_back(temp);
        vrn.push_back(0.001);

        (*SDatabase)[(*ptpairsCMin)[j]].id = Map3D->map.size();
	idVec[(*ptpairsCMin)[j]] = Map3D->map.size();
        (*pointsCMin)[j].x = (*pointsCMin)[j].x + RoverTrajectory->tEST[*STEP].x;
        (*pointsCMin)[j].y = (*pointsCMin)[j].y + RoverTrajectory->tEST[*STEP].y;
        (*pointsCMin)[j].z = (*pointsCMin)[j].z + RoverTrajectory->tEST[*STEP].z;

        Map3D->addLandmark(Landmark(Map3D->map.size(), (*pointsCMin)[j], (idx - 3) / 3, 0.05));

        idx = idx + 3;
      }
      else
      {
        // Update EIF
//ROS_INFO("Update Landmark");
        ids.push_back(Map3D->map[idVec[(*ptpairsCMin)[j]]].matPos);

        vrf.push_back(Map3D->map[idVec[(*ptpairsCMin)[j]]].stddev);


        vf.push_back(temp);
      }
    }

    slam_filter_ptr_->recoverMean();
    slam_filter_ptr_->augment(vn, vrn);
    slam_filter_ptr_->update(vf, vrf, &ids);
    slam_filter_ptr_->recoverMean();


    RoverTrajectory->tEST[*STEP].x = (*slam_filter_ptr_->mu).at<double>(0, 0);
    RoverTrajectory->tEST[*STEP].y = (*slam_filter_ptr_->mu).at<double>(1, 0);
    RoverTrajectory->tEST[*STEP].z = (*slam_filter_ptr_->mu).at<double>(2, 0);

    // Publishing the rover's estimated position in ROS
    PMSLAM_Data_msg.TrajectoryOut.x = RoverTrajectory->tEST[*STEP].x;
    PMSLAM_Data_msg.TrajectoryOut.y = RoverTrajectory->tEST[*STEP].y;
    PMSLAM_Data_msg.TrajectoryOut.z = RoverTrajectory->tEST[*STEP].z;

//ROS_INFO("(%f, %f, %f)",PMSLAM_Data_msg.TrajectoryOut.x,PMSLAM_Data_msg.TrajectoryOut.y,PMSLAM_Data_msg.TrajectoryOut.z);

    // update map
    //#seg fault----------

    for (unsigned int i = 0; i < SDatabase->size(); ++i)
    {
      if ((idVec[i] > 0) && (idVec[i]< (int)Map3D->map.size()))
      {
        //ROS_INFO("IDIDID: %d\nsizesize: %d",(*SURFDatabase)[i].id, Map3D->map.size());
        //wrong id in database
        id = Map3D->map[idVec[i]].matPos * 3 + 3;

        Map3D->map[idVec[i]].position.x = (*slam_filter_ptr_->mu).at<double>(id, 0);
        Map3D->map[idVec[i]].position.y = (*slam_filter_ptr_->mu).at<double>(id + 1, 0);
        Map3D->map[idVec[i]].position.z = (*slam_filter_ptr_->mu).at<double>(id + 2, 0);

        // Assigning data to the feature position and gtruth vectors
        PMSLAM_Data_msg.MapOut.ID.push_back(idVec[i]);
        PMSLAM_Data_msg.MapOut.positions.y.push_back(Map3D->map[idVec[i]].position.x);
        PMSLAM_Data_msg.MapOut.positions.x.push_back(Map3D->map[idVec[i]].position.y);
        PMSLAM_Data_msg.MapOut.positions.z.push_back(Map3D->map[idVec[i]].position.z);
      }
    }
  }

  //remove re-mappings
  RoverTrajectory->tEST.clear();
  delete RoverTrajectory;
}

