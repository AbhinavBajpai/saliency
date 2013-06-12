/*
 * file_seq_pub_core.cpp
 *
 * Created on: 19 Jul 2012
 * Author: Piotr Weclewski (weclewski.piotr@gmail.com)
 *
 * Definition of node responsible for read offline data and publish image and odometry as a real rover
 */

#include "file_seq_pub_core.h"
#include "dynamic/rosdyp.h"

FileSeqPubCore::FileSeqPubCore(ros::NodeHandle *_n) :
    it_(*_n)
{
  // Initialise node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate_, double(1));

  private_node_handle.param("pub_image_topic_name", pub_image_topic_name_, string("cam_image"));
  private_node_handle.param("pub_odom_topic_name", pub_odom_topic_name_, string("odom"));

  private_node_handle.param("loop", loop_f_, bool(false));
  private_node_handle.param("one_frame", one_frame_, int(-1));

  string _tmppath = ros::package::getPath("sscrovers_file_sequence_publisher");

  //add '/' wjen missed at the end of path
  private_node_handle.param("path_to_images", path_to_images_, string(_tmppath + "/input/"));
  if (*path_to_images_.rbegin() != '/') //last element get by reverse_begin iterator
    path_to_images_.push_back('/');

  private_node_handle.param("image_file_name_prefix", image_file_name_prefix_, string("frame_orig_"));

  private_node_handle.param("start_no", start_no_, int(1000000));

  private_node_handle.param("image_extension", image_extension_, string(".jpg"));
  //add '.' when missed in extension
  if (image_extension_[0] != '.')
    image_extension_.insert(0, ".");

  private_node_handle.param("path_to_trajectory", path_to_poses_, string(_tmppath + "/input/"));
  //add '/' wjen missed at the end of path
  if (*path_to_poses_.rbegin() != '/') //last element get by reverse_begin iterator
    path_to_poses_.push_back('/');

  private_node_handle.param("trajectory_file_name", trajectory_file_name_, string("trajectory.fli"));

  image_pub_ = it_.advertise(pub_image_topic_name_.c_str(), 0);
  odom_pub_ = _n->advertise<nav_msgs::Odometry>(pub_odom_topic_name_.c_str(), 0);

  cv_img_ptr_.reset(new cv_bridge::CvImage);

  step_ = -1;

  loadTrajectory();
}

FileSeqPubCore::~FileSeqPubCore()
{

}

void FileSeqPubCore::process()
{
  if (one_frame_ == -1) //play back set of frames
  {
    step_++;
  }
  else //play back only one chosen frame
  {
    step_ = one_frame_;
    if (step_ > (int)temp_trajectory_.size())
    {
      ROS_ERROR("Chosen frame is not exist!");
      ros::shutdown();
    }
  }

  if (step_ < (int)temp_trajectory_.size()) //process step
  {
    ROS_INFO("---------------Processing step %d...---------------\n", step_);
    acquirePose();
    acquireImage();
    publishOdom();
    publishImage();
  }
  else //if end of loaded data
  {
    if (loop_f_) //if want to play from begginning
    {
      ROS_INFO("----Start from the beginning...----\n");
      step_ = -1;
    }
    else //end of publishing
    {
      ROS_INFO("----Whole trajectory was published. Quitting...----\n");
      ros::shutdown();
    }
  }
}

void FileSeqPubCore::loadTrajectory()
{
  //open file
  std::string traj_path = path_to_poses_ + trajectory_file_name_;
  ifstream inputfile(traj_path.c_str());
  if (!inputfile.is_open())
  {
    ROS_ERROR("Could not open a trajectory file at: %s. Quitting...", traj_path.c_str());
    ros::shutdown();
  }

  char const row_delim = '\n';
  char const field_delim = ' ';

  for (string row; getline(inputfile, row, row_delim);)
  {
    temp_trajectory_.push_back(Rows_t::value_type());
    istringstream ss(row);
    for (string field; getline(ss, field, field_delim);)
    {
      temp_trajectory_.back().push_back(field);
    }
  }

  //! reading problem
  if (temp_trajectory_.size() == 0)
  {
    ROS_WARN("There was a problem with reading %s file. Quitting...", traj_path.c_str());
    ros::shutdown();
  }
  else
    ROS_INFO("trajectory file size = %u.", (unsigned int)temp_trajectory_.size());
}

void FileSeqPubCore::acquirePose()
{
  //! parse file content
  if (step_ < (int)temp_trajectory_.size())
  {
    double _y = 0.0, _p = 0.0, _r = 0.0;

    sscanf(temp_trajectory_[step_][1].c_str(), "%lf", &odom_msg_.pose.pose.position.x);
    sscanf(temp_trajectory_[step_][2].c_str(), "%lf", &odom_msg_.pose.pose.position.y);
    sscanf(temp_trajectory_[step_][3].c_str(), "%lf", &odom_msg_.pose.pose.position.z);
    sscanf(temp_trajectory_[step_][5].c_str(), "%lf", &_y);
    sscanf(temp_trajectory_[step_][6].c_str(), "%lf", &_p);
    sscanf(temp_trajectory_[step_][7].c_str(), "%lf", &_r);

    odom_msg_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(_r, _p, _y);
  }
  else
  {
    ROS_ERROR("There is no more trajectory points... Quitting...");
    ros::shutdown();
  }

}

std::string FileSeqPubCore::makeFileName(const std::string& basename, const int& index, const std::string& ext)
{
  ostringstream result;
  result << basename << index << ext;
  return result.str();
}

void FileSeqPubCore::acquireImage()
{
  int stepn = start_no_ + step_;

  //build a full path
  std::string filename = path_to_images_ + makeFileName(image_file_name_prefix_, stepn, image_extension_);

  ROS_INFO("image file: %s", filename.c_str());

  //needed for ros image transport
  cv_img_ptr_->encoding = "mono8";

  //OpenCV read from file
  try
  {
    cv_img_ptr_->image = cv::imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
  }
  catch (cv::Exception& e)
  {
    ROS_ERROR("Image did not load correctly! Exception: /t %s", e.what());
  }
}

void FileSeqPubCore::publishImage()
{
  //TODO remove dependency on step
  //cv_img_ptr_->header.stamp = ros::Time::now();
  cv_img_ptr_->header.stamp.sec = step_;
  cv_img_ptr_->header.stamp.nsec = step_;
  image_pub_.publish(cv_img_ptr_->toImageMsg());
}

void FileSeqPubCore::publishOdom()
{
  //TODO remove dependency on step
  //odom_.header.stamp = ros::Time::now();
  odom_msg_.header.stamp.nsec = step_;
  odom_pub_.publish(odom_msg_);
}

//-----------------------------MAIN-----------------------------

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "file_seq_pub");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  FileSeqPubCore *file_pub_core = new FileSeqPubCore(&n);

  ros::Rate r(file_pub_core->rate_);

  // Main loop.
  while (n.ok())
  {
    file_pub_core->process();
    ros::spinOnce();//was commented!?
    r.sleep();
  }

  return 0;
}
