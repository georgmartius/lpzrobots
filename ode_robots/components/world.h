#include <math.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <iostream>

#include <vector>
#include <list>

#include "vector.h"
#include "exception.h"
#include "cubic_spline.h"



#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

#ifndef world_h
#define world_h



namespace university_of_leipzig {
namespace robot {


class IComponent;

typedef std::list<IComponent*> ComponentContainer;



class WorldDescription {
 public:
  WorldDescription();

  Vector3<dReal> v3_gravity;
};


class EarthWorldDescription : public WorldDescription {
 public:
  EarthWorldDescription();
};


class World {
 protected:
  dWorldID world_id;
  dSpaceID space_id;
  dJointGroupID joint_group_id_contact;

  ComponentContainer component_container;

 public:
  World(const WorldDescription &r_desc); // = WorldDescription());
  ~World();


  dWorldID get_world_id() const;
  dSpaceID get_space_id() const;

  dJointGroupID get_joint_group_id_contact() const;

  bool collision_callback(dGeomID geom_id_0, dGeomID geom_id_1);

  
  void step(dReal step_size);
  void draw() const;

  // this function should only be called from components
  // which register themselves in the world
  ComponentContainer::iterator register_component(IComponent &r_component);

  //  void unregister_component(const ComponentContainer::iterator &r_it);
};



}
}



#endif
