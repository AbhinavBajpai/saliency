
/** This file defines the Landmark class and related functions.

*	@date: 09/11/2010
*	@author Karin Shala
*
*
*/

#ifndef LANDMARK_H
#define LANDMARK_H

#include "opencv/cv.h"
#include <vector>
using namespace cv;


/** @class Landmark.h "src/Landmark.h"
 *  @brief This is the Landmark class.
 *
 *  @detail The Landmark class defines a landmark with a specific 3D position.
 */
class Landmark
{

public:

	/** Default Constructor
    */
	Landmark();

	/** Constructor taking arguments for initialisation.
	* @param[in] indexinit
	* @param[in] positioninit
	* @param[in] matPosinit
    */
	Landmark( int indexinit, CvPoint3D64f positioninit, int matPosinit, double stddevInit );

	/** Destructor
    */
	~Landmark();

	/** Class member defining the index of the landmark.
    */
	int index;

	/** Class member defining the 3D world position of the landmark.
    */
	CvPoint3D64f position;

	/** Class member defining the position of the landmark in the SLAM filter information matrix.
    */
	int matPos; 

	/** Class member defining the standard deviation of the landmark as a confidence value.
    */
	double stddev;

};



#endif
