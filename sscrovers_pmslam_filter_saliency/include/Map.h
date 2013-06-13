/** This file defines the Map class and related functions.

*	@date: 09/11/2010
*	@author Karin Shala
*
*
*/

#ifndef MAP_H
#define MAP_H

#include "Landmark.h"
#include <vector>

/** @class Map.h "src/Map.h"
 *  @brief This is the Map class.
 *
 *  @detail The Map class defines a map of 3D landmarks.
 */
class Map
{

public:

	/** Default Constructor
    */
	Map();

	/** Destructor
    */
	~Map();

	/** Class member defining the map as a vector of landmarks.
    */
	vector<Landmark> map;

	void addLandmark(Landmark lm);

};

#endif
