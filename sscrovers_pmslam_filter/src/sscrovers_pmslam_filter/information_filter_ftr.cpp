#include "information_filter_ftr.h"

InformationFilterFtr::InformationFilterFtr(int* step, RoverState *s, FeaturesDB_t* db) :
    curr_pose_ptr_(s), step_ptr_(step), db_ptr_(db)
{
  slam_filter_ptr_ = new EIF();
  map3d_ptr_ = new Map();
}

InformationFilterFtr::~InformationFilterFtr()
{
  delete slam_filter_ptr_;
  delete map3d_ptr_;
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
  RoverTrajectory->tEST.resize(*step_ptr_ + 1);
  RoverTrajectory->tEST[*step_ptr_] = *curr_pose_ptr_;
  int*STEP = step_ptr_;
  vector<SURFPoint>* SURFDatabase = db_ptr_->storage_;

  int id;
  vector<CvPoint3D64f> vn, vf;
  vector<double> vrn, vrf;
  vector<int> ids;
  int idx = slam_filter_ptr_->idx;

  PMSLAM_Data_msg.MapOut.ID.clear();
  PMSLAM_Data_msg.MapOut.positions.x.clear();
  PMSLAM_Data_msg.MapOut.positions.y.clear();
  PMSLAM_Data_msg.MapOut.positions.z.clear();

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
    for (unsigned int j = 1; j < ptpairsCMin->size(); j += 2)
    {
      //temp.x = (*pointsCMin)[(j-1)/2].x + RoverTrajectory->tEST[*STEP].x;
      //temp.y = (*pointsCMin)[(j-1)/2].y + RoverTrajectory->tEST[*STEP].y;
      //temp.z = (*pointsCMin)[(j-1)/2].z + RoverTrajectory->tEST[*STEP].z;

      //temp.x = (*pointsCMin)[(j-1)/2].x - RoverTrajectory->tTRUE[*STEP].x;
      //temp.y = (*pointsCMin)[(j-1)/2].y - RoverTrajectory->tTRUE[*STEP].y;
      //temp.z = (*pointsCMin)[(j-1)/2].z - RoverTrajectory->tTRUE[*STEP].z;

      temp.x = (*pointsCMin)[(j - 1) / 2].x;
      temp.y = (*pointsCMin)[(j - 1) / 2].y;
      temp.z = (*pointsCMin)[(j - 1) / 2].z;

      if ((*SURFDatabase)[(*ptpairsCMin)[j - 1]].id == 0) // if new landmark
      {
        //vn.push_back((*pointsCMin)[(j-1)/2]);
        vn.push_back(temp);
        vrn.push_back(0.001);

        (*SURFDatabase)[(*ptpairsCMin)[j - 1]].id = Map3D->map.size();
        (*pointsCMin)[(j - 1) / 2].x = (*pointsCMin)[(j - 1) / 2].x + RoverTrajectory->tEST[*STEP].x;
        (*pointsCMin)[(j - 1) / 2].y = (*pointsCMin)[(j - 1) / 2].y + RoverTrajectory->tEST[*STEP].y;
        (*pointsCMin)[(j - 1) / 2].z = (*pointsCMin)[(j - 1) / 2].z + RoverTrajectory->tEST[*STEP].z;

        Map3D->addLandmark(Landmark(Map3D->map.size(), (*pointsCMin)[(j - 1) / 2], (idx - 3) / 3, 0.05));
        //Map3D->addLandmark(Landmark( Map3D->map.size(), temp, (idx-3)/3, 0.001 ));
        idx = idx + 3;
      }
      else
      {
        // Update EIF
        ids.push_back(Map3D->map[(*SURFDatabase)[(*ptpairsCMin)[j - 1]].id].matPos);

        //vf.push_back((*pointsCMin)[(j-1)/2]);

        vrf.push_back(Map3D->map[(*SURFDatabase)[(*ptpairsCMin)[j - 1]].id].stddev);

        //temp.x = Map3D->map[(*SURFDatabase)[(*ptpairsCMin)[j-1]].id].position.x - RoverTrajectory->tEST[*STEP].x;
        //temp.y = Map3D->map[(*SURFDatabase)[(*ptpairsCMin)[j-1]].id].position.y - RoverTrajectory->tEST[*STEP].y;
        //temp.z = Map3D->map[(*SURFDatabase)[(*ptpairsCMin)[j-1]].id].position.z - RoverTrajectory->tEST[*STEP].z;

        //temp.x = (*SURFDatabase)[(*ptpairsCMin)[j-1]].gtruth[0] - RoverTrajectory->tTRUE[*STEP].x;
        //temp.y = (*SURFDatabase)[(*ptpairsCMin)[j-1]].gtruth[1] - RoverTrajectory->tTRUE[*STEP].y;
        //temp.z = (*SURFDatabase)[(*ptpairsCMin)[j-1]].gtruth[2] - RoverTrajectory->tTRUE[*STEP].z;

        vf.push_back(temp);
      }
    }
    //gettimeofday(&start, NULL);
    slam_filter_ptr_->recoverMean();
    slam_filter_ptr_->augment(vn, vrn);
    slam_filter_ptr_->update(vf, vrf, &ids);
    slam_filter_ptr_->recoverMean();
    //gettimeofday(&finish, NULL);
    //seconds = finish.tv_sec - start.tv_sec;
    //useconds = finish.tv_usec - start.tv_usec;
    //mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
    //ROS_INFO("Processed slam_filter_ptr_ in %ld MSecs", mtime);

    RoverTrajectory->tEST[*STEP].x = (*slam_filter_ptr_->mu).at<double>(0, 0);
    RoverTrajectory->tEST[*STEP].y = (*slam_filter_ptr_->mu).at<double>(1, 0);
    RoverTrajectory->tEST[*STEP].z = (*slam_filter_ptr_->mu).at<double>(2, 0);

    /*
     ROS_INFO("\nRoverTrajectory[%d]:"
     "\n   x = %f \t     y = %f \t    z = %f "
     "\n yaw = %f \t pitch = %f \t roll = %f", *STEP, RoverTrajectory->tEST[*STEP].x,
     RoverTrajectory->tEST[*STEP].y,
     RoverTrajectory->tEST[*STEP].z,
     RoverTrajectory->tEST[*STEP].yaw,
     RoverTrajectory->tEST[*STEP].pitch,
     RoverTrajectory->tEST[*STEP].roll);
     */
    // Publishing the rover's estimated position in ROS
    PMSLAM_Data_msg.TrajectoryOut.x = RoverTrajectory->tEST[*STEP].x;
    PMSLAM_Data_msg.TrajectoryOut.y = RoverTrajectory->tEST[*STEP].y;
    PMSLAM_Data_msg.TrajectoryOut.z = RoverTrajectory->tEST[*STEP].z;

    // update map
    //#seg fault----------

    for (unsigned int i = 0; i < SURFDatabase->size(); ++i)
    {
      if (((*SURFDatabase)[i].id > 0) && ((*SURFDatabase)[i].id < (int)Map3D->map.size()))
      {
        //ROS_INFO("IDIDID: %d\nsizesize: %d",(*SURFDatabase)[i].id, Map3D->map.size());
        //wrong id in database
        id = Map3D->map[(*SURFDatabase)[i].id].matPos * 3 + 3;

        Map3D->map[(*SURFDatabase)[i].id].position.x = (*slam_filter_ptr_->mu).at<double>(id, 0);
        Map3D->map[(*SURFDatabase)[i].id].position.y = (*slam_filter_ptr_->mu).at<double>(id + 1, 0);
        Map3D->map[(*SURFDatabase)[i].id].position.z = (*slam_filter_ptr_->mu).at<double>(id + 2, 0);

        // Assigning data to the feature position and gtruth vectors
        PMSLAM_Data_msg.MapOut.ID.push_back((*SURFDatabase)[i].id);
        PMSLAM_Data_msg.MapOut.positions.y.push_back(Map3D->map[(*SURFDatabase)[i].id].position.x);
        PMSLAM_Data_msg.MapOut.positions.x.push_back(Map3D->map[(*SURFDatabase)[i].id].position.y);
        PMSLAM_Data_msg.MapOut.positions.z.push_back(Map3D->map[(*SURFDatabase)[i].id].position.z);
      }
    }
  }

  //remove re-mappings
  RoverTrajectory->tEST.clear();
  delete RoverTrajectory;
}

