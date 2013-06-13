/** This file defines the SURFPoint class and related functions.

*	@date: 09/11/2010
*	@author Karin Shala
*
*
*/

#ifndef SURFPOINT_H
#define SURFPOINT_H

#include <cv.h>
#include <vector>
#include "RoverState.h"


/** @class SURFPoint.h "src/SURFPoint.h"
 *  @brief This is the SURFPoint class.
 *
 *  @detail The SURFPoint class defines the interest point used for PMSLAM.
 */
class SURFPoint
{

public:

	/** Class member for unique index of the SURF point.
	*/
	int index;

	/** Class member for SURF keypoint.
	*/
	CvSURFPoint pt; 

	/** Class member for SURF descriptor.
	*/
	float descriptor[64];

	/** Class member for the rover state the SURF point was observed at.
	*/
	RoverState state;

	/** Class member for the location of the SURF point in the last observed image.
	*/
	int pxLocation[2];

	/** Class member for the SURF point's ground truth in 3D world coordinates.
	*/
	float gtruth[3]; 

	/** Class member counting the number of times the SURF point is observed.
	*/
	int n;

	/** Class member for index of corresponding landmark in the map (0 if no landmark is assigned).
	*/
	int id;

	/** Class member indicating the last step at which the SURF point is observed.
	*/
	int step;

	/** Class member indicating if (0) the SURF point was not observed in the previous itteration or
	*							   (1) the SURF point was observed in the previous itteration or
	*							   (2) the SURF point was not observed in the previous 10 itterations.
	*/
	int flag;


	/** Default Constructor.
	*/
	SURFPoint();

	/** Constructor taking arguments for initialisation.
	*/
	SURFPoint( int indexinit, CvSURFPoint ptinit, float descriptorinit[64], RoverState stateinit, float gtruthinit[3], int ninit, int idinit, int stepinit, int flaginit);
	
	/** Destructor.
	*/
	~SURFPoint();

	/** Member function for acquiring a SURF point's 3D ground truth from PANGU.
	* 
	* @param[in] sock socket for PANGU connection
	* @return void
	*
	*/
	//void getGroundTruth(SOCKET sock);


};


#endif
