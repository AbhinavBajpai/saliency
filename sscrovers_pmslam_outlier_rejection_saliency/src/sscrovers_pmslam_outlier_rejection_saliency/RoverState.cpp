#include "RoverState.h"

RoverState::RoverState()
{
  x = 0.0;
  y = 0.0;
  z = 0.0;
  yaw = 0.0;
  pitch = 0.0;
  roll = 0.0;
}

// RoverState::~RoverState()
// {}

RoverState::RoverState(double xinit, double yinit, double zinit, double yawinit, double pitchinit, double rollinit)
{
  x = xinit;
  y = yinit;
  z = zinit;
  yaw = yawinit;
  pitch = pitchinit;
  roll = rollinit;

}

RoverState::RoverState(double stateinit[6])
{
  x = stateinit[0];
  y = stateinit[1];
  z = stateinit[2];
  yaw = stateinit[3];
  pitch = stateinit[4];
  roll = stateinit[5];

}

RoverState RoverState::operator-(const RoverState & state)
{
  return RoverState(x - state.x, y - state.y, z - state.z, yaw, pitch, roll);
  //return RoverState(x-state.x, y-state.y, z-state.z, yaw-state.yaw, pitch-state.pitch, roll-state.roll);
}
