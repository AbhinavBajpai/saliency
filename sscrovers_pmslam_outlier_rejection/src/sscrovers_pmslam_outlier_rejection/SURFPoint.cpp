#include "SURFPoint.h"

#include <iostream>

SURFPoint::SURFPoint()
{
  pxLocation[0]=pxLocation[1]=0;//added

    index = 0;
    pt.pt.x = 0.0;
    pt.pt.y = 0.0;
    pt.dir = 0.0;
    pt.hessian = 0.0;
    pt.laplacian = 0;
    pt.size = 0;

    state.x = 0.0;
    state.y = 0.0;
    state.z = 0.0;
    state.yaw = 0.0;
    state.pitch = 0.0;
    state.roll = 0.0;

    for (int i = 0; i < 3; i++)
    {
      gtruth[i] = 0.0;
    }

    for (int i = 0; i < 64; i++)
    {
      descriptor[i] = 0.0;

    }

    //cworld[3] = cworldinit[3];
    n = 0;
    id = 0;
    step = 0;
    flag = 0;
}

SURFPoint::~SURFPoint()
{
}

SURFPoint::SURFPoint(int indexinit, CvSURFPoint ptinit, float descriptorinit[64], RoverState stateinit,
                     float gtruthinit[3], int ninit, int idinit, int stepinit, int flaginit)
{
  pxLocation[0]=pxLocation[1]=0;//added

  index = indexinit;
  pt = ptinit;

  state.x = stateinit.x;
  state.y = stateinit.y;
  state.z = stateinit.z;
  state.yaw = stateinit.yaw;
  state.pitch = stateinit.pitch;
  state.roll = stateinit.roll;

  for (int i = 0; i < 3; i++)
  {
    gtruth[i] = gtruthinit[i];
  }

  for (int i = 0; i < 64; i++)
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
