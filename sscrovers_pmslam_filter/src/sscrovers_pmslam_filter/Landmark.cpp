#include "Landmark.h"

Landmark::Landmark()
{}

Landmark::~Landmark()
{}

Landmark::Landmark( int indexinit, CvPoint3D64f positioninit, int matPosinit, double stddevInit )
{
	index = indexinit;
	position.x = positioninit.x;
	position.y = positioninit.y;
	position.z = positioninit.z;
	matPos = matPosinit;
	stddev = stddevInit; 

}