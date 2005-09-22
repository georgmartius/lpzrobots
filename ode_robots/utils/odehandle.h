#ifndef __ODEHANDLE_H
#define __ODEHANDLE_H

#include <ode/common.h>

/** Data structure for accessing the ODE */
typedef struct OdeHandle
{
  OdeHandle( ) { }
  OdeHandle(  dWorldID _world, dSpaceID _space, dJointGroupID _jointGroup){
    world = _world; space = _space; jointGroup= _jointGroup;
  }
  dWorldID world;
  dSpaceID space;
  dJointGroupID jointGroup;
} OdeHandle;

#endif
