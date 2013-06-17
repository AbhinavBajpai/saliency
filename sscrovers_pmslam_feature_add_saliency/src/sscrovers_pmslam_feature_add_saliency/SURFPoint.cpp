
#include "SURFPoint.h"

SURFPoint::SURFPoint()
{}


SURFPoint::~SURFPoint()
{}


SURFPoint::SURFPoint( int indexinit, CvSURFPoint ptinit, float descriptorinit[64], RoverState stateinit, float gtruthinit[3], int ninit, int idinit, int stepinit, int flaginit)
{
	index = indexinit;
	pt = ptinit;

	state.x = stateinit.x;
	state.y = stateinit.y;
	state.z = stateinit.z;
	state.yaw = stateinit.yaw;
	state.pitch = stateinit.pitch;
	state.roll = stateinit.roll;

	for (int i=0; i<3; i++)
	{
	gtruth[i] = gtruthinit[i];
	}

	for (int i=0; i<64; i++)
	{
		descriptor[i] = descriptorinit[i];

	}

	//cworld[3] = cworldinit[3];
	n = ninit;
	id = idinit;
	step = stepinit;
	flag = flaginit;
}

//void SURFPoint::getGroundTruth(SOCKET sock)
//{
//
//	int px = 512, py = 512;
//	float uPG = 0, vPG = 0, xw = 0, yw = 0, zw = 0;
//	char err = 0;
//	//CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(image2S.keypoints, i);
//	uPG = pt.pt.x/px;
//	vPG = 1-pt.pt.y/py;
//	pan_protocol_lookup_point(sock, uPG, vPG, &xw, &yw, &zw, &err);
//	//float gtruth[3] = {xw, yw, zw};
//
//	gtruth[0] = xw;
//	gtruth[1] = yw;
//	gtruth[2] = zw;
//
//	//xw1 = SURFDatabase[ptpairs[j-1]].gtruth[0];
//	//yw1 = SURFDatabase[ptpairs[j-1]].gtruth[1];
//	//zw1 = SURFDatabase[ptpairs[j-1]].gtruth[2];
//
//}
