#include "filter_keypoints_or.h"

FilterKeypointsOR::FilterKeypointsOR(int *step, RoverState* curr_pos, FeaturesDB_t* db) :
   mode_(2), px_(0), py_(0), disp_f_(0), save_f_(0),lm_track_(0), scale_(0), keypoints_ptr_(NULL),descriptors_ptr_(NULL), curr_pose_ptr_(curr_pos), step_(step), surf_db_ptr_(db)
{

}

FilterKeypointsOR::~FilterKeypointsOR()
{

}

void FilterKeypointsOR::addToDatabase(vector<SURFPoint> *SURFDatabase, RoverState *state, int px, int py)
{
  /******************variables re-maping for compatibility with PM_SLAM********************/
  CvSeq* keypoints = keypoints_ptr_;
  CvSeq* descriptors = descriptors_ptr_;
  /****************************************************************************************/

  int STEP = *step_;

  for (int i = 0; i < keypoints->total; i++)
  {
    CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(keypoints, i);
    float gtruth[3] = {0, 0, 0};
    (*SURFDatabase).push_back(SURFPoint(i, *r, (float*)cvGetSeqElem(descriptors, i), *state, gtruth, 1, 0, STEP, 0));
  }
}

void FilterKeypointsOR::matchDB(vector<SURFPoint>* SURFDatabase, vector<int>& ptpairs, int px, int py)
{
  /******************variables re-maping for compatibility with PM_SLAM********************/
  CvSeq* keypoints = keypoints_ptr_;
  CvSeq* descriptors = descriptors_ptr_;
  //***************************************************************************************/

  //float u1PG = 0, v1PG = 0, xw1 = 0, yw1 = 0, zw1 = 0;
  //char err = 0;

  findPairs(SURFDatabase, keypoints, descriptors, ptpairs);
}

Mat findFundamentalMat_local(const Mat& points1, const Mat& points2, int method, double param1, double param2, vector<
    uchar>* mask)
{
  CV_Assert(points1.checkVector(2) >= 0 && points2.checkVector(2) >= 0 &&
      (points1.depth() == CV_32F || points1.depth() == CV_32S) &&
      points1.depth() == points2.depth())
;  Mat F(3, 3, CV_64F);
  CvMat _pt1 = Mat(points1), _pt2 = Mat(points2);
  CvMat matF = F, _mask, *pmask = 0;
  if( mask )
  {
    mask->resize(points1.cols*points1.rows*points1.channels()/2);
    pmask = &(_mask = cvMat(1, (int)mask->size(), CV_8U, (void*)&(*mask)[0]));
  }
  int n = cvFindFundamentalMat( &_pt1, &_pt2, &matF, method, param1, param2, pmask );
  if( n <= 0 )
  F = Scalar(0);
  return F;
}

void FilterKeypointsOR::rejectOutliers(vector<SURFPoint>* SURFDatabase, vector<int>& ptpairs)
{
  /******************variables re-maping for compatibility with PM_SLAM********************/
  CvSeq* keypoints = keypoints_ptr_;
  //CvSeq* descriptors = descriptors_ptr_;
  /****************************************************************************************/

  int j = 0;
  vector<uchar> outliers;
  vector<int> pts;
  vector<Point2f> points1, points2;

  int n = ptpairs.size() / 2;
  pts.clear();
  outliers.clear();

  if (n > 0)//PW added
  {
    for (int i = 0; i < n; i++)
    {
      points1.push_back((*SURFDatabase)[ptpairs[j]].pt.pt);
      CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(keypoints, ptpairs[j + 1]);
      points2.push_back(r->pt);
      j += 2;
    }

    Mat F = findFundamentalMat_local(Mat(points1), Mat(points2), FM_RANSAC, 1, 0.99, &outliers);
    j = 0;

    for (int i = 0; i < n; i++)
    {
      //char OL = outliers[i];
      if (outliers[i] == 1)
      {
        pts.push_back(ptpairs[j]);
        pts.push_back(ptpairs[j + 1]);
      }
      j += 2;
    }
    ptpairs = pts;
  }
}

void FilterKeypointsOR::filterKeypoints()
{
  /******************variables re-maping for compatibility with PM_SLAM********************/
  FilterKeypointsOR* imageFKin = this;
  vector<int> *ptpairsFKout = &pt_pairs_;
  vector<SURFPoint> *SURFDatabase = surf_db_ptr_->storage_;
  typedef struct
  {
    vector<RoverState> tEST;
  } RoverTrajectory_t;
  RoverTrajectory_t* RoverTrajectory = new RoverTrajectory_t;
  //RoverTrajectory->tEST = new vector<RoverState>;
  RoverTrajectory->tEST.resize((*step_) + 1);
  RoverTrajectory->tEST[*step_] = *curr_pose_ptr_;
#define keypoints keypoints_ptr_
#define descriptors descriptors_ptr_
  //step_++;
  int *STEP = step_;
  int *PX = &px_;
  int *PY = &py_;
  bool *DISP = &disp_f_;
  bool *SAVE = &save_f_;
  int *LM_TRACK = &lm_track_;
  int *SCALE = &scale_;
  int *MODE = &mode_;

  /***************************************************************************/

  //char err = 0;

  //*************************************** test ***************************************

  //float xw2 = 0, yw2 = 0, zw2 = 0;
  //float u2PG = 0, v2PG = 0;
  //const double PI = 3.141592;
  //int px = *PX;
  //int py = *PY;
  //double x, y, u, v;

  //double VFOV = 52.2*PI/180;
  //double HFOV = 66.7*PI/180;
  //double pan  = 1;
  //double tilt = 41;
  //double CamH = 0.52;

  //ofstream outputFile;
  //outputFile.open (make_filename("output/depthtest_", *STEP, ".txt").c_str());

  ////for (int j=0;j<imageFKin->keypoints->total;j++)
  ////{
  ////	CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(imageFKin->keypoints, j);
  ////	u2PG = r->pt.x/px;
  ////	v2PG = 1-r->pt.y/py;
  ////	pan_protocol_lookup_point(*sock, u2PG, v2PG, &xw2, &yw2, &zw2, &err);

  ////	outputFile << r->pt.x << "\t" << r->pt.y << "\t" << xw2 << "\t" << yw2  << "\t" << zw2 << "\n";
  ////}

  //for (int j=0;j<imageFKin->keypoints->total;j++)
  //{
  //	CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(imageFKin->keypoints, j);

  //	u	 = r->pt.x/px;
  //	v	 = r->pt.y/py;
  //	u2PG = r->pt.x/px;
  //	v2PG = 1-r->pt.y/py;
  //	//pan_protocol_lookup_point(*sock, u2PG, v2PG, &xw2, &yw2, &zw2, &err);

  //	//y = 1/tan((20*PI/180) + atan((2*v-1)*tan((30*PI/180)/2)));
  //	//x = y * (2*u-1)*tan((30*PI/180)/2);
  //	y = ((*RoverTrajectory).tEST[*STEP].z+CamH)/tan((((*RoverTrajectory).tEST[*STEP].pitch+tilt)*PI/180)+atan((2*v-1)*tan(VFOV/2)));
  //	x = y*tan((((*RoverTrajectory).tEST[*STEP].yaw+pan)*PI/180)+atan((2*u-1)*tan(HFOV/2)));

  //	outputFile << "xp=" << "\t" << r->pt.x << "\t" << "yp=" << "\t" << r->pt.y << "\t" << "u=" << "\t" << u << "\t" << "v="
  //			   << "\t" << v << "\t" <<  "x=" << "\t" << x << "\t" << "y=" << "\t" << y << "\n"; // << xw2 << "\t" << yw2  <<  "\t" << abs(x-xw2) <<  "\t" << abs(y - yw2) << "\n";
  //}

  //outputFile.close();

  //************************************************************************************

  float gtruth[3] = {0, 0, 0}; // ground truth not available from real imagery
  bool found;
  //float u1PG = 0, v1PG = 0, xw1 = 0, yw1 = 0, zw1 = 0;

  if (SURFDatabase->empty())//was '*STEP==0'
  {
    imageFKin->addToDatabase(SURFDatabase, &RoverTrajectory->tEST[*STEP], *PX, *PY);
  }
  else
  {
    imageFKin->matchDB(SURFDatabase, *ptpairsFKout, *PX, *PY);
    for (int i = 0; i < imageFKin->keypoints->total; i++)
    {
      //CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(imageFKin->keypoints, i);
      found = false;

      for (unsigned int j = 1; j < ptpairsFKout->size(); j += 2)
      {
        if ((*ptpairsFKout)[j] == i) // if found
        {
          found = true;
          break;
        }
        else
          found = false;
      }

      if (found == false)
      {
        CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(imageFKin->keypoints, i);

        SURFDatabase->push_back(SURFPoint(SURFDatabase->size(), *r, (float*)cvGetSeqElem(imageFKin->descriptors, i),
                                          RoverTrajectory->tEST[*STEP], gtruth, 1, 0, *STEP, 0));
      }
    }

    if (*DISP)
      imageFKin->display(true, SURFDatabase, ptpairsFKout, *SCALE, *LM_TRACK);

    imageFKin->rejectOutliers(SURFDatabase, *ptpairsFKout);

    if (*DISP)
      imageFKin->display(true, SURFDatabase, ptpairsFKout, *SCALE, *LM_TRACK);
    if (*SAVE)
      imageFKin->save(true, SURFDatabase, ptpairsFKout, *SCALE, *LM_TRACK, *MODE);

    // Update SURFDatabase with matched keypoints
    for (unsigned int j = 1; j < ptpairsFKout->size(); j += 2)
    {
      //CvSURFPoint* r = (CvSURFPoint*) cvGetSeqElem(imageFKin->keypoints, (*ptpairsFKout)[j]); // Retrieve matched keypoint from current image
      //float dX = (*r).pt.x - (*SURFDatabase)[(*ptpairsFKout)[j - 1]].pt.pt.x;
      //float dY = (*r).pt.y - (*SURFDatabase)[(*ptpairsFKout)[j - 1]].pt.pt.y;
      //double lineLength = sqrt(dX * dX + dY * dY);
      if (*STEP == 12)
      {
        ;//cout<<"pause";//ROS_INFO("pause"); - here is no ros dependency
      }
      // Determine how to set the flag based on last citation of keypoint
      //			if ((*SURFDatabase)[(*ptpairsFKout)[j-1]].step == *STEP-1)
      //				(*SURFDatabase)[(*ptpairsFKout)[j-1]].flag = 1;			// re-observed landmark from the last image
      //			else if ((*SURFDatabase)[(*ptpairsFKout)[j-1]].step <= *STEP-20)
      //				(*SURFDatabase)[(*ptpairsFKout)[j-1]].flag = 2;			// re-observed landmark from an image more than 20 iterations old
      //			else if ((*SURFDatabase)[(*ptpairsFKout)[j-1]].step <= *STEP-40)
      //				(*SURFDatabase)[(*ptpairsFKout)[j-1]].flag = 3;			// re-observed landmark from an image more than 40 iterations old
      //			else
      //				(*SURFDatabase)[(*ptpairsFKout)[j-1]].flag = 0;			// re-observed landmark within the last 20
      //
      //			(*SURFDatabase)[(*ptpairsFKout)[j-1]].pt.pt = (*r).pt;									// update landmark with last seen pixel location
      //			(*SURFDatabase)[(*ptpairsFKout)[j-1]].n = (*SURFDatabase)[(*ptpairsFKout)[j-1]].n++;	// Increment citation counter
      //			(*SURFDatabase)[(*ptpairsFKout)[j-1]].step = *STEP;										// update landmark with last seen STEP

    }

    imageFKin->rejectOutliers(SURFDatabase, *ptpairsFKout);

    if (*DISP)
      imageFKin->display(true, SURFDatabase, ptpairsFKout, *SCALE, *LM_TRACK);

    // Update SURFDatabase with matched keypoints

    for (unsigned int j = 1; j < ptpairsFKout->size(); j += 2)
    {

      CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(imageFKin->keypoints, (*ptpairsFKout)[j]); // Retrieve matched keypoint from current image
      //float dX = (*r).pt.x - (*SURFDatabase)[(*ptpairsFKout)[j - 1]].pt.pt.x;
      //float dY = (*r).pt.y - (*SURFDatabase)[(*ptpairsFKout)[j - 1]].pt.pt.y;
      //double lineLength = sqrt(dX * dX + dY * dY);
      if (*STEP == 12)
      {
        ;//cout<<"pause";//ROS_INFO("pause"); - here is no ros dependency
      }
      // Determine how to set the flag based on last citation of keypoint
      if ((*SURFDatabase)[(*ptpairsFKout)[j - 1]].step == *STEP - 1)
        (*SURFDatabase)[(*ptpairsFKout)[j - 1]].flag = 1; // re-observed landmark from the last image
      else if ((*SURFDatabase)[(*ptpairsFKout)[j - 1]].step <= *STEP - 20)
        (*SURFDatabase)[(*ptpairsFKout)[j - 1]].flag = 2; // re-observed landmark from an image more than 20 iterations old
      else if ((*SURFDatabase)[(*ptpairsFKout)[j - 1]].step <= *STEP - 40)
        (*SURFDatabase)[(*ptpairsFKout)[j - 1]].flag = 3; // re-observed landmark from an image more than 40 iterations old
      else
        (*SURFDatabase)[(*ptpairsFKout)[j - 1]].flag = 0; // re-observed landmark within the last 20

      (*SURFDatabase)[(*ptpairsFKout)[j - 1]].pt.pt = (*r).pt; // update landmark with last seen pixel location
      (*SURFDatabase)[(*ptpairsFKout)[j - 1]].n = (*SURFDatabase)[(*ptpairsFKout)[j - 1]].n++; // Increment citation counter
      (*SURFDatabase)[(*ptpairsFKout)[j - 1]].step = *STEP; // update landmark with last seen STEP
    }

  }

}

void FilterKeypointsOR::display(bool showkp, vector<SURFPoint> *SURFDatabase, vector<int> *ptpairsCurrent,
                                int ImgScale, int lmTracking)
{
}

void FilterKeypointsOR::save(bool showkp, vector<SURFPoint> *SURFDatabase, vector<int> *ptpairsCurrent, int ImgScale,
                             int lmTracking, int mode)
{
}
