/*
 * direct_depth_dp.cpp
 *
 *  Created on: 9 Jul 2012
 *      Author: sscrovers
 */

#include "triangulate_dp.h"

TriangulateDP::TriangulateDP()
{
	// TODO Auto-generated constructor stub
}

TriangulateDP::~TriangulateDP()
{
	// TODO Auto-generated destructor stub
}

void TriangulateDP::transformAngles(float yaw, float pitch, float roll, double *alpha, double *beta, double *gamma)
{
	const double PI = 3.141592;
	//*alpha = yaw*PI/180;
	//*beta = 2*PI-pitch*PI/180;
	//*gamma = roll*PI/180;

	*alpha = roll*PI/180;
	*beta = pitch*PI/180;
	*gamma = yaw*PI/180;
}

double TriangulateDP::absoluteDistance(double x, double y, double z)
{
	double r;
	r = sqrt(x*x + y*y + z*z);
	return r;
}

void TriangulateDP::triangulate()
{
//surfdatabase access??
//void directDepth(vector<SURFPoint>* SURFDatabase, vector<int>& ptpairsin, vector<int>& ptpairsout, RoverState *state, vector<CvPoint3D64f> &points3D, double rng)
//{
/*
	int px = 512; double d;
	CvPoint3D64f pt3D;
	vector<int> pts;

	const double PI = 3.141592;

	pts.clear();
	points3D.clear();

	double u, v;
	double x, y, z, alpha, beta, gamma; // yaw, pitch, roll in radians

	for (int j=1;j<ptpairsin.size();j+=2)
	{
		CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(keypoints, ptpairsin[j]);

		u = r->pt.x/px; 
		v = r->pt.y/px;

//		y = state->z/tan(((-state->pitch)*PI/180)+atan((2*v - 1)*tan((30*PI/180)/2)));
		y = state->z/tan(((state->pitch)*PI/180)+atan((2*v - 1)*tan((30*PI/180)/2)));

		x = y * (2*u - 1)*tan((30*PI/180)/2);

		CvMat Pt, *transMat, PR, PT, *RR, *TT;
		
		transformAngles(state->yaw, state->pitch, state->roll, &alpha, &beta, &gamma); // yaw pitch roll

		Mat point(3, 1, CV_64FC1);

		point.at<double>(0,0) = x;
		point.at<double>(1,0) = y;
		point.at<double>(2,0) = 0;

		double sina = sin(alpha);
		double sinb = sin(beta);
		double sinc = sin(gamma);
		double cosa = cos(alpha);
		double cosb = cos(beta);
		double cosc = cos(gamma);

		//// translation matrix
		//Mat T = Mat::zeros(3, 1, CV_64F);
		//T.at<double>(0,0) = state->x;
		//T.at<double>(1,0) = state->y;
		//T.at<double>(2,0) = state->z;

		// rotation matrix
		Mat R = Mat::zeros(3, 3, CV_64F);
		R.at<double>(0,0) = cosa*cosc - sina*sinb*sinc;
		R.at<double>(1,0) = -cosb*sinc;
		R.at<double>(2,0) = cosa*sinb*sinc - cosc*sina;
		R.at<double>(0,1) = cosa*sinc - cosc*sina*sinb;
		R.at<double>(1,1) = cosb*cosc;
		R.at<double>(2,1) = - sina*sinc - cosa*cosc*sinb;
		R.at<double>(0,2) = cosb*sina;
		R.at<double>(1,2) = sinb;
		R.at<double>(2,2) = cosa*cosb;

		// rotation from camera into world coordinate system
		Mat pointW = R.inv()*point;

		pt3D.x = pointW.at<double>(0,0);
		pt3D.y = pointW.at<double>(1,0);
		pt3D.z = 0; // not used

		d = absoluteDistance(pt3D.x, pt3D.y, pt3D.z);

		if (d < rng)
		{
			points3D.push_back(pt3D);
			pts.push_back(ptpairsin[j-1]);
			pts.push_back(ptpairsin[j]);
		}

	}

	ptpairsout = pts;
*/
}

/*
void TriangulateDP::triangulate(vector<SURFPoint>* SURFDatabase, vector<int>& ptpairsin, vector<int>& ptpairsout, RoverState *state, vector<CvPoint3D64f> &points3D, double rng)
{
	CvMat *points1GL, *points2GL, *projMat1, *projMat2, *points4D;
	float xt = 0, yt = 0, zt = 0, wt = 0; 
	int px = 512; double d, b;
	CvPoint3D64f pt3D;
	vector<int> pts;

	pts.clear();
	points3D.clear();

	RoverState rel;

	float u1PG = 0, v1PG = 0, xw1 = 0, yw1 = 0, zw1 = 0;
	char err = 0;

	srand(time(NULL));

	for (int j=1;j<ptpairsin.size();j+=2)
	{
		CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(keypoints, ptpairsin[j]);
		u1PG = r->pt.x/px; 
		v1PG = 1-r->pt.y/px;
//		pan_protocol_lookup_point(sock, u1PG, v1PG, &xw1, &yw1, &zw1, &err);

		double s = double(rand())/(double(RAND_MAX)+1.0); // random double in range 0.0 to 1.0 (non inclusive)
		double noise = -(0.001) + s*((0.001) + (0.001)); // transform to wanted range

		pt3D.x = xw1 + noise;
		pt3D.y = yw1 + noise;
		pt3D.z = zw1 + noise;
		
		//points3D.push_back(pt3D);


		//b = sqrt( pow((*SURFDatabase)[ptpairsin[j-1]].state.x - (*state).x, 2) 
		//	+ pow((*SURFDatabase)[ptpairsin[j-1]].state.y - (*state).y, 2)
		//	+ pow((*SURFDatabase)[ptpairsin[j-1]].state.z - (*state).z, 2) );

		////check if enough 'baseline' for triangulation
		//if (b >= 0.5)
		//{

		//	CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(keypoints, ptpairsin[j]);
		//	//u2PG = r->pt.x/px; 
		//	//v2PG = 1-r->pt.y/px;

		//	projMat1 = cvCreateMat(3, 4, CV_64FC1);
		//	//createProjectionMatrix(&(*SURFDatabase)[ptpairsin[j-1]].state, projMat1);
		//	rel = (*SURFDatabase)[ptpairsin[j-1]].state-(*state);
		//	createProjectionMatrix(&rel, projMat1);


		//	//introduce position error for depth perception test
		//	//float stateerr[6];
		//	//stateerr[0] = state[0] + 0;
		//	//stateerr[1] = state[1] + 0;
		//	//stateerr[2] = state[2] + 0;
		//	//stateerr[3] = state[3] + 0;
		//	//stateerr[4] = state[4] + 0;
		//	//stateerr[5] = state[5] + 0;

		//	projMat2 = cvCreateMat(3, 4, CV_64FC1);
		//	//createProjectionMatrix(state, projMat2);
		//	createProjectionMatrix(&RoverState(0,0,0,state->yaw,state->pitch,state->roll), projMat2);

		//	points1GL = cvCreateMat(2, 1, CV_64FC1);
		//	points2GL = cvCreateMat(2, 1, CV_64FC1);
		//	points4D = cvCreateMat(4, 1, CV_32FC1);

		//	cvmSet(points1GL, 0, 0, ((*SURFDatabase)[ptpairsin[j-1]].pt.pt.x/px*2-1));
		//	cvmSet(points1GL, 1, 0, (1-(*SURFDatabase)[ptpairsin[j-1]].pt.pt.y/px*2));
		//	cvmSet(points2GL, 0, 0, (r->pt.x/px*2-1));
		//	cvmSet(points2GL, 1, 0, (1-r->pt.y/px*2));

		//	cvTriangulatePoints(projMat1, projMat2, points1GL, points2GL, points4D);

		//	xt = cvmGet(points4D,0,0);
		//	yt = cvmGet(points4D,1,0);
		//	zt = cvmGet(points4D,2,0);
		//	wt = cvmGet(points4D,3,0);

		//	pt3D.x = xt/wt;
		//	pt3D.z = yt/wt;
		//	pt3D.y = zt/wt;

			//d = absoluteDistance(pt3D.x-state->x, pt3D.y-state->z, pt3D.z-state->y);
			d = absoluteDistance(pt3D.x, pt3D.y, pt3D.z);


			if (d < rng)
			{
				points3D.push_back(pt3D);
				pts.push_back(ptpairsin[j-1]);
				pts.push_back(ptpairsin[j]);
			}

		//}

	}

	ptpairsout = pts;

}
*/
