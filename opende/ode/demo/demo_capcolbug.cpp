/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 * Simulation by Georg Martius (C) 2011 georg dot martius at web dot de  *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include <ode-dbl/ode.h>
#include <drawstuff/drawstuff.h>
#include <assert.h>
#include <unistd.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawLine dsDrawLineD
#endif

#define CAP // use capsules, otherwise use cylinders
// #define CAP_CAP // check chapsule-capsule collisions  instead of with box
// some constants
#define SIDE   (5.0f)	// side length of a capsule/cyl
#ifdef CAP
#define HEIGHT (1.0f)	// side length of a capsule
#else
#define HEIGHT (2.0f)	// side length of a cylinder
#endif
#define RADIUS (0.5f)	// side length of a capsule/cyl
#define MASS (1.0)	// mass of a box
#define NUMCAPS 5	// number of capsules/cylinders

// positions of the capsules/cylinders (columwise)
dReal x[5] = {0,   2.5,   -2.5,  -3.3,  0};
dReal y[5] = {0,   0,     0,     -2,    2};
dReal z[5] = {5,   4,     4,     4,     5.5};

// dynamics and collision objects
static dSpaceID space;
static dWorldID world;
static dGeomID biggeom;
static dBodyID body[NUMCAPS];
static dGeomID geom[NUMCAPS];
static dJointGroupID contactgroup;

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  assert(o1);
  assert(o2);

  const int N = 32;
  dContact contact[N];
  int n = dCollide (o1,o2,N,&(contact[0].geom),sizeof(dContact));
  if (n > 0) 
  {
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {0.02,0.02,0.02};

    for (int i=0; i<n; i++) 
    {
      contact[i].surface.slip1    = 0.7;
      contact[i].surface.slip2    = 0.7;
      contact[i].surface.mode     = dContactSoftERP | dContactSoftCFM | 
        dContactApprox1 | dContactSlip1 | dContactSlip2;
      contact[i].surface.mu       = 2;
      contact[i].surface.soft_erp = 0.9 ;
      contact[i].surface.soft_cfm = 0.1 ;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
		    dGeomGetBody(contact[i].geom.g1),
		    dGeomGetBody(contact[i].geom.g2));

      dsSetColor (1,0,0);
      dsDrawBox (contact[i].geom.pos,RI,ss);
      dVector3 n;
      dsSetColor (1,0,1);
      for (int j=0; j<3; j++) 
        n[j] = contact[i].geom.pos[j] + 0.3*contact[i].geom.normal[j];
      dsDrawLine (contact[i].geom.pos,n);
    }
  }
}

// start simulation - set viewpoint
static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);
  static float xyz[3] = {0.1638,-7.2281,6.2400};
  static float hpr[3] = {90.0000,-25.0000,0.0000};
  dsSetViewpoint (xyz,hpr);
}


// simulation loop
static void simLoop (int pause)
{
  if(!pause){
    dSpaceCollide (space,0,&nearCallback);
    dWorldStep (world,0.0001);
    dJointGroupEmpty (contactgroup);
  }  
  dsSetDrawMode(1);
  dsSetColor (0,1,1);
  for(int i=0; i < NUMCAPS ; i++){
#ifdef CAP
    dsDrawCapsule(dBodyGetPosition(body[i]),dBodyGetRotation(body[i]), HEIGHT, RADIUS);
#else
    dsDrawCylinder(dBodyGetPosition(body[i]),dBodyGetRotation(body[i]), HEIGHT, RADIUS); 
#endif
  }
  dsSetTexture (DS_WOOD);
  dsSetDrawMode(0);
  dsSetColor (1,1,0);
  dsSetColorAlpha (1,1,0,.3);
#ifdef CAP_CAP  
    dsDrawCapsule(dGeomGetPosition(biggeom),dGeomGetRotation(biggeom), SIDE, SIDE*.5);
#else
  dReal sides1[3] = {SIDE,SIDE,SIDE};
  dsDrawBox (dGeomGetPosition(biggeom),dGeomGetRotation(biggeom),sides1);
#endif

  usleep(100000);// make it very slow to see what happens
}


int main (int argc, char **argv)
{
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version          = DS_VERSION;
  fn.start            = &start;
  fn.step             = &simLoop;
  fn.command          = 0;
  fn.stop             = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  // create world
  dInitODE2(0);
  world        = dWorldCreate();
  space        = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,0);

   
#ifdef CAP_CAP  
  biggeom = dCreateCCylinder(space, SIDE*.5, SIDE);
#else
  biggeom = dCreateBox (space, SIDE, SIDE, SIDE);
#endif
  dGeomSetPosition(biggeom, 0,0,3);   
  
  dQuaternion q;
  dQFromAxisAndAngle (q,0,1,0,M_PI*0.5);
  dMass m;
  dMassSetCapsule(&m, 1.0, 3 , RADIUS, HEIGHT); 
  dMassAdjust (&m, MASS); 
  
  for(int i=0; i<NUMCAPS; i++){
    body[i] = dBodyCreate (world);
    dBodySetMass (body[i],&m);
    dBodySetPosition (body[i],x[i],y[i],z[i]);
    if(i>0)
      dBodySetQuaternion (body[i],q);
#ifdef CAP
    geom[i] = dCreateCCylinder (space, RADIUS, HEIGHT);
#else
    geom[i] = dCreateCylinder (space, RADIUS, HEIGHT);
#endif
    dGeomSetBody (geom[i], body[i]); 
  }  

  // run simulation
  dsSimulationLoop (argc,argv,352,288,&fn);

  dJointGroupEmpty (contactgroup);
  dJointGroupDestroy (contactgroup);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}
