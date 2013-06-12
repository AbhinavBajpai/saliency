/**
 *	@date: 28/10/2010
 *	@author Karin Shala
 *
 *	@detail This file defines the rover/camera position and orientation.
 *
 *
 *
 */

#ifndef ROVERSTATE_H
#define ROVERSTATE_H

/** @class RoverState.h "src/RoverState.h"
 *  @brief This is the RoverState class.
 *
 *  @detail The RoverState class defines the 6D rover/camera state (position and orientation).
 */

class RoverState
{

public:

  //! Default constructor.
  RoverState();

  //! Default destructor.
  ~RoverState();

  /** Constructor taking arguments for initialisation.
   * @param[in] xinit
   * @param[in] yinit
   * @param[in] zinit
   * @param[in] yawinit
   * @param[in] pitchinit
   * @param[in] rollinit
   */
  RoverState(double xinit, double yinit, double zinit, double yawinit, double pitchinit, double rollinit);

  /** Constructor taking arguments for initialization.
   * @param[in] stateinit
   */
  RoverState(double stateinit[6]);

  RoverState operator-(const RoverState &);

  /** Class members for rover position and orientation.
   */
  double x, y, z, yaw, pitch, roll;

};

#endif
