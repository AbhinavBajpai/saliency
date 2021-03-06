//===================================================================================
// Name        : saliencyDetectionHou.cpp
// Author      : Oytun Akman, oytunakman@gmail.com
// Editor	   : Joris van de Weem, joris.vdweem@gmail.com (Conversion to ROS)
// Version     : 1.2
// Copyright   : Copyright (c) 2010 LGPL
// Description : C++ implementation of "Saliency Detection: A Spectral Residual 
//				 Approach" by Xiaodi Hou and Liqing Zhang (CVPR 2007).												  
//===================================================================================
// v1.1: Changed Gaussianblur of logamplitude to averaging blur and gaussian kernel of saliency map to sigma = 8, kernelsize = 5
//      for better consistency with the paper. (Joris)
// v1.2: Ported to Robot Operating System (ROS) (Joris)

#include <saliency_detection/saliencyDetectionHou.h>


void saliencyMapHou::imageCB(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	cv_bridge::CvImagePtr cv_ptr;
	sensor_msgs::Image salmap_, heatmap_, temp_;
	geometry_msgs::Point salientpoint_;
	geometry_msgs::Point32 temppoint_;
	geometry_msgs::PoseArray poseArray;
	sscrovers_pmslam_common::featureMap fm_;

	Mat image_, saliencymap_, heatMap, image2_;
	Point pt_salient;
	double maxVal;


	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg_ptr, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	cv_ptr->image.copyTo(image_);
	cv::Size sizeall = image_.size();
	image2_ = image_.clone();

	saliencymap_.create(image_.size(),CV_8UC1);
	saliencyMapHou::calculateSaliencyMap(&image_, &saliencymap_);
	resize(saliencymap_, saliencymap_, sizeall, 0, 0, INTER_LINEAR);

	//-- Return most salient point --//
	cv::minMaxLoc(saliencymap_,NULL,&maxVal,NULL,&pt_salient);
	salientpoint_.x = pt_salient.x;
	salientpoint_.y = pt_salient.y;

	
	//	CONVERT FROM CV::MAT TO ROSIMAGE FOR PUBLISHING
	saliencymap_.convertTo(saliencymap_, CV_8UC1,255);

	//affan's and guy's addition

	threshold(saliencymap_, saliencymap_, 0, 255, cv::THRESH_TOZERO|cv::THRESH_OTSU);

	heatMap = saliencymap_.clone();

        threshold(saliencymap_, saliencymap_, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);

	//bounding boxes

	cv::Mat clonedImage,clonedImage2,clonedImage3, cropped;
	clonedImage = saliencymap_.clone();
	clonedImage2 = saliencymap_.clone();
	std::vector< std::vector<cv::Point> > contours;
	std::vector<cv::Point> points;
	cv::findContours(clonedImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	for (size_t i=0; i<contours.size(); i++) {
		for (size_t j = 0; j < contours[i].size(); j++) {
			cv::Point p = contours[i][j];
			points.push_back(p);
		}
		if(points.size() > 0){
			cv::Rect brect = cv::boundingRect(cv::Mat(points).reshape(2));
			//cv::Rect brect1(brect.x-,brect.y-2,brect.width+2,brect.height+2);
			cropped = clonedImage2(brect);
			

			geometry_msgs::Pose poser;
			poser.position.x = brect.x + brect.width/2;
			poser.position.y = brect.y + brect.height/2;

			temppoint_.x = brect.x;
			temppoint_.y = brect.y;
			fillImage(temp_, "mono8",cropped.rows, cropped.cols, cropped.step, const_cast<uint8_t*>(cropped.data));
			fm_.points.push_back(temppoint_);
			fm_.imgs.push_back(temp_);
			fm_.number++;
			
			poser.orientation.x = brect.x;
			poser.orientation.y = brect.y;
			poser.orientation.z = brect.height;
			poser.orientation.w = brect.width;
			poseArray.header.stamp = cv_ptr->header.stamp;
			poseArray.poses.push_back(poser); 
			cv::rectangle(saliencymap_, brect.tl(), brect.br(), cv::Scalar(100, 100, 200), 6, CV_AA);
                        cv::rectangle(image2_, brect.tl(), brect.br(), cv::Scalar(0, 0, 255), 6, CV_AA);
			cv::circle(image2_,cv::Point(poser.position.x,poser.position.y),3,cv::Scalar(255, 0, 0), 4, CV_AA);
		}
		points.clear();
	}
		
	//adaptiveThreshold(saliencymap_, saliencymap_, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 5 ,0);

	fillImage(salmap_, "bgr8",image2_.rows, image2_.cols, image2_.step, const_cast<uint8_t*>(image2_.data));
	fillImage(heatmap_, "mono8",heatMap.rows, heatMap.cols, heatMap.step, const_cast<uint8_t*>(heatMap.data));
	poseArray.header.stamp = ros::Time::now();
	saliencymap_pub_.publish(salmap_);
	point_pub_.publish(salientpoint_);
	heatmap_pub_.publish(heatmap_);
	poseArray.header =   cv_ptr->header;
	features_pub_.publish(poseArray);
	featureMap_pub_.publish(fm_);

	return;
}


void saliencyMapHou::calculateSaliencyMap(const Mat* src, Mat* dst)
{
	Mat grayTemp, grayDown;
	vector<Mat> mv;	
	//Size imageSize(160,120);
	Size imageSize(64,64);
	Mat realImage(imageSize,CV_64F);
	Mat imaginaryImage(imageSize,CV_64F); imaginaryImage.setTo(0);
	Mat combinedImage(imageSize,CV_64FC2);
	Mat imageDFT;	
	Mat logAmplitude;
	Mat angle(imageSize,CV_64F);
	Mat magnitude(imageSize,CV_64F);
	Mat logAmplitude_blur;
	
	cvtColor(*src, grayTemp, CV_BGR2GRAY);
	resize(grayTemp, grayDown, imageSize, 0, 0, INTER_LINEAR);
	for(int j=0; j<grayDown.rows;j++)
       	for(int i=0; i<grayDown.cols; i++)
       		realImage.at<double>(j,i) = grayDown.at<uchar>(j,i);
			
	mv.push_back(realImage);
	mv.push_back(imaginaryImage);	
	merge(mv,combinedImage);	
	dft( combinedImage, imageDFT);
	split(imageDFT, mv);	

	//-- Get magnitude and phase of frequency spectrum --//
	cartToPolar(mv.at(0), mv.at(1), magnitude, angle, false);
	log(magnitude,logAmplitude);	
	//-- Blur log amplitude with averaging filter --//
	blur(logAmplitude, logAmplitude_blur, Size(3,3), Point(-1,-1), BORDER_DEFAULT);
	
	exp(logAmplitude - logAmplitude_blur,magnitude);
	//-- Back to cartesian frequency domain --//
	polarToCart(magnitude, angle, mv.at(0), mv.at(1), false);
	merge(mv, imageDFT);
	dft( imageDFT, combinedImage, CV_DXT_INVERSE); 
	split(combinedImage, mv);

	cartToPolar(mv.at(0), mv.at(1), magnitude, angle, false);
	GaussianBlur(magnitude, magnitude, Size(5,5), 2.5, 2.5, BORDER_DEFAULT);
	magnitude = magnitude.mul(magnitude);

	double minVal,maxVal;
	minMaxLoc(magnitude, &minVal, &maxVal);
	magnitude = magnitude / maxVal;

	Mat tempFloat(imageSize,CV_32F);
	for(int j=0; j<magnitude.rows;j++)
       		for(int i=0; i<magnitude.cols; i++)
       			tempFloat.at<float>(j,i) = magnitude.at<double>(j,i);

	resize(magnitude, *dst, dst->size(), 0, 0, INTER_LINEAR);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "saliencymap");

	saliencyMapHou salmapHou;

	ros::spin();

	return 0;
}
