#include "world.h"
#include "component.h"

namespace university_of_leipzig {
namespace robot {


/**
 * WorldDescription
 *
 *
 */
WorldDescription::WorldDescription()
{
  v3_gravity = Vector3<dReal>(static_cast<dReal>(0), 
			      static_cast<dReal>(0),
			      static_cast<dReal>(-9.81));
}



EarthWorldDescription::EarthWorldDescription()
{
  v3_gravity = Vector3<dReal>(static_cast<dReal>(0), 
			      static_cast<dReal>(0),
			      static_cast<dReal>(-9.81));

}


void __collision_callback(void *p_data, dGeomID geom_id_0, dGeomID geom_id_1)
{
  // cast p_data to an world object
  World *p_world = reinterpret_cast<World*>(p_data);

  p_world->collision_callback(geom_id_0, geom_id_1);
}


/**
 * World
 *
 *
 */
World::World(const WorldDescription &r_desc = EarthWorldDescription())
{
  // create an ode world
  world_id = dWorldCreate();

  // create the space
  space_id = dHashSpaceCreate(0);


  // create the joint group for contact joints
  joint_group_id_contact = dJointGroupCreate(0);


  dWorldSetGravity(world_id,
		   r_desc.v3_gravity.x,
		   r_desc.v3_gravity.y,
		   r_desc.v3_gravity.z);
}


/**
 * ~World
 *
 *
 */
World::~World()
{
  // check if there are still components registered
  // since this is a destructor raising an exception
  // would terminate the program (afaik)
  // anyway we simply display an error message
  if(0 != component_container.size())
    std::cerr << "WARNING: the world was destroyed - even though there were \
                  still components registered.\n";

  dJointGroupDestroy(joint_group_id_contact);
  dSpaceDestroy(space_id);
  dWorldDestroy(world_id);
}


/**
 * World::get_world_id()
 *
 *
 */
dWorldID World::get_world_id() const
{
  return world_id;
}


/**
 * World::get_space_id()
 *
 *
 */
dSpaceID World::get_space_id() const
{
  return space_id;
}



/**
 * World::get_joint_group_id_contact()
 *
 *
 */
dJointGroupID World::get_joint_group_id_contact() const
{
  return joint_group_id_contact;
}


/**
 * World::collsision_callback()
 *
 *
 */
bool World::collision_callback(dGeomID geom_id_0, dGeomID geom_id_1)
{
  // try passing this collission to all (registered) components
  // if none of them treated the collission either raise an exception
  // or perform some default collission handling
  bool b_collision_treated;
  
  ComponentContainer::const_iterator it = component_container.begin();

  for(it; it != component_container.end(); ++it) {
    b_collision_treated = 
      (*it)->collision_callback(this, geom_id_0, geom_id_1);

    if(true == b_collision_treated)
      return true;
  }

  // none of the components' collision handlers accepted the collision
  // what now? (except for running screaming into a wall)

  //  std::cout << "collission ignored\n";
  return false;
}



/**
 * World::step()
 *
 *
 */
void World::step(dReal step_size)
{
  //std::cout << "schni\n";
  dSpaceCollide   (space_id, this, &__collision_callback);
  //std::cout << "schna\n";
  dWorldStep      (world_id, step_size);
  //std::cout << "schnappi\n";
  dJointGroupEmpty(joint_group_id_contact);
  //qstd::cout << "blubb\n";


  // notify all world objects (sensors) that time has advanced
}



/**
 * World::draw()
 *
 *
 */
void World::draw() const
{
  ComponentContainer::const_iterator it = component_container.begin();

  for(it; it != component_container.end(); ++it)
    (*it)->draw();

}


/**
 * World::register_component()
 *
 *
 */
ComponentContainer::iterator World::register_component(IComponent &r_component)
{
  return component_container.insert(component_container.end(), &r_component);
}



}
}
