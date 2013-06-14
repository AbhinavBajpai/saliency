//===================================================================================
// Name        : saliencyDetectionRudinac.cpp
// Author      : Joris van de Weem, joris.vdweem@gmail.com
// Version     : 1.1
// Copyright   : Copyright (c) 2011 LGPL
// Description : C++ implementation of "Maja Rudinac, Pieter P. Jonker. 
// 				"Saliency Detection and Object Localization in Indoor Environments". 
//				ICPR'2010. pp.404~407											  
//===================================================================================
// v1.1: Ported to Robot Operating System (ROS)

#include <saliency_detection/saliencyDetectionRudinac.h>

void saliencyMapRudinac::imageCB(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	cv_bridge::CvImagePtr cv_ptr;
	sensor_msgs::Image salmap_, heatmap_;
	geometry_msgs::Point salientpoint_;
	geometry_msgs::PoseArray poseArray;

	Mat image_, image2_, saliencymap_;
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
	saliencyMapRudinac::calculateSaliencyMap(&image_, &saliencymap_);
	resize(saliencymap_, saliencymap_, sizeall, 0, 0, INTER_LINEAR);

	//-- Return most salient point --//
	cv::minMaxLoc(saliencymap_,NULL,&maxVal,NULL,&pt_salient);
	salientpoint_.x = pt_salient.x;
	salientpoint_.y = pt_salient.y;

	int erosion_size = 15;   
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                      cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1), 
                      cv::Point(erosion_size, erosion_size) );
	cv::dilate(saliencymap_, saliencymap_, element); 
	cv::erode(saliencymap_, saliencymap_, element); 

	//	CONVERT FROM CV::MAT TO ROSIMAGE FOR PUBLISHING
	saliencymap_.convertTo(saliencymap_, CV_8UC1,255);

	threshold(saliencymap_, saliencymap_, 0, 255, cv::THRESH_TOZERO|cv::THRESH_OTSU);
	cv::Mat heatMap;
	heatMap = saliencymap_.clone();

        threshold(saliencymap_, saliencymap_, 0, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);

	cv::Mat clonedImage;
	clonedImage = saliencymap_.clone();
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
			geometry_msgs::Pose poser;
			poser.position.x = brect.x + brect.width/2;
			poser.position.y = brect.y + brect.height/2;
			poser.orientation.x = brect.x;
			poser.orientation.y = brect.y;
			poser.orientation.z = brect.height;
			poser.orientation.w = brect.width;
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
	binarymap_pub_.publish(heatmap_);
	features_pub_.publish(poseArray);
	
}


void saliencyMapRudinac::calculateSaliencyMap(const Mat* src, Mat* dst)
{
	Size imageSize(128,128);
	Mat srcDown(imageSize,CV_64F);
	Mat magnitudeI(imageSize,CV_64F);
	Mat magnitudeRG(imageSize,CV_64F);
	Mat magnitudeBY(imageSize,CV_64F);
	Mat magnitude(imageSize,CV_64F);
	
	resize(*src, srcDown, imageSize, 0, 0, INTER_LINEAR);
	
	createChannels(&srcDown);
	createSaliencyMap(I,&magnitudeI);
	createSaliencyMap(RG,&magnitudeRG);
	createSaliencyMap(BY,&magnitudeBY);
	
	magnitude= (magnitudeI + magnitudeRG + magnitudeBY);
	GaussianBlur(magnitude, magnitude, Size(11,11), 2.5, 2.5, BORDER_DEFAULT);

	//-- Scale to domain [0,1] --//
	double minVal,maxVal;
	minMaxLoc(magnitude, &minVal, &maxVal);
	magnitude = magnitude / maxVal;

	resize(magnitude, *dst, dst->size(), 0, 0, INTER_LINEAR);
}


void saliencyMapRudinac::createChannels(const Mat* src)
{
	
	b.create(src->size(),CV_32F);
	g.create(src->size(),CV_32F);
	r.create(src->size(),CV_32F);
	I.create(src->size(),CV_32F);
	vector<Mat> planes;	
	split(*src, planes);
	Mat rgmax(src->size(),CV_32F);
	Mat rgbmax(src->size(),CV_32F);
	Mat mask(src->size(),CV_32F);

	for(int j=0; j<r.rows;j++)
       	for(int i=0; i<r.cols; i++)
       	{
			b.at<float>(j,i) = planes[0].at<uchar>(j,i);
			g.at<float>(j,i) = planes[1].at<uchar>(j,i);
			r.at<float>(j,i) = planes[2].at<uchar>(j,i);
       	}
	
	I = r+g+b;
	//threshold(I, I, 255, 255, THRESH_TRUNC); // Saturation as in Matlab?
	I = I/3;
		
	rgmax = max(r,g);
	rgbmax = max(rgmax,b);
	
	//-- Prevent that the lowest value is zero, because you cannot divide by zero.
	for(int j=0; j<r.rows;j++)
       	for(int i=0; i<r.cols; i++)
       	{
			if (rgbmax.at<float>(j,i) == 0) rgbmax.at<float>(j,i) = 1;
       	}


	RG = abs(r-g)/rgbmax;
	BY = abs(b - min(r,g))/rgbmax;

	rgbmax = rgbmax/255;
	//-- If max(r,g,b)<0.1 all components should be zero to stop large fluctuations of the color opponency values at low luminance --//
	threshold(rgbmax,mask,.1,1,THRESH_BINARY);
	RG = RG.mul(mask);
	BY = BY.mul(mask);
	I = I.mul(mask);
}



void saliencyMapRudinac::createSaliencyMap(const Mat src, Mat* dst)
{
	vector<Mat> mv;	
	
	Mat realImage(src.size(),CV_64F);
	Mat imaginaryImage(src.size(),CV_64F); imaginaryImage.setTo(0);
	Mat combinedImage(src.size(),CV_64FC2);
	Mat image_DFT;	
	Mat logAmplitude;
	Mat angle(src.size(),CV_64F);
	Mat Magnitude(src.size(),CV_64F);
	Mat logAmplitude_blur;
	
	for(int j=0; j<src.rows;j++){
       	for(int i=0; i<src.cols; i++){
       		realImage.at<double>(j,i) = src.at<float>(j,i);
       	}
	}

			
	mv.push_back(realImage);
	mv.push_back(imaginaryImage);	
	merge(mv,combinedImage);	
	
	dft( combinedImage, image_DFT);
	split(image_DFT, mv);	

	//-- Get magnitude and phase of frequency spectrum --//
	cartToPolar(mv.at(0), mv.at(1), Magnitude, angle, false);
	log(Magnitude,logAmplitude);	
		
	//-- Blur log amplitude with averaging filter --//
	blur(logAmplitude, logAmplitude_blur, Size(3,3), Point(-1,-1), BORDER_DEFAULT);
	exp(logAmplitude - logAmplitude_blur,Magnitude);
	
	polarToCart(Magnitude, angle,mv.at(0), mv.at(1),false);
	merge(mv,image_DFT);
	dft(image_DFT,combinedImage,CV_DXT_INVERSE);

	split(combinedImage,mv);
	cartToPolar(mv.at(0), mv.at(1), Magnitude, angle, false);
	Magnitude = Magnitude.mul(Magnitude);

	Mat tempFloat(src.size(),CV_32F);
	for(int j=0; j<Magnitude.rows;j++)
       	for(int i=0; i<Magnitude.cols; i++)
       		tempFloat.at<float>(j,i) = Magnitude.at<double>(j,i);
	
	*dst = tempFloat;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "saliencymap");

	saliencyMapRudinac salmapRudinac;

	ros::spin();

	return 0;
}
