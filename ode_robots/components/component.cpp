#include "./component.h"
#include "world.h"

namespace university_of_leipzig {
namespace robot {


/*****************************************************************************/
/* AbstractMotorComponent                                                    */
/*****************************************************************************/
AbstractMotorComponent::AbstractMotorComponent(dJointID _joint_id) :
  joint_id(_joint_id)
{
}



/*****************************************************************************/
/* UniversalMotorComponent                                                   */
/*****************************************************************************/
UniversalMotorComponent::
UniversalMotorComponent(dJointID _joint_id, char _axis) :
  AbstractMotorComponent(_joint_id),
  axis(_axis),
  wire(this)
{
  if(dJointTypeUniversal != dJointGetType(joint_id))
    exception::InvalidArgument().raise();

  if(axis != 0 && axis != 1)
    exception::InvalidArgument().raise();
}


void UniversalMotorComponent::
set_angular_velocity(dReal angular_velocity)
{
  //  std::cout << angular_velocity << "\n";
  if(0 == axis) {
    dJointSetUniversalParam (joint_id ,dParamVel,  angular_velocity * 70);
    dJointSetUniversalParam (joint_id, dParamFMax, 2.5);		
  }
  else if(1 == axis) {
    dJointSetUniversalParam (joint_id ,dParamVel2, angular_velocity * 70);
    dJointSetUniversalParam (joint_id, dParamFMax2, 2.5);
  }
  else
    std::cerr << "shouldnt happen\n";
}


dReal UniversalMotorComponent::
get_angular_velocity() const
{
  if(0 == axis)
    return dJointGetUniversalAngle1(joint_id);
  else if(1 == axis)
    return dJointGetUniversalAngle2(joint_id);
  else
    std::cerr << "shouldnt happen\n";
}


unsigned UniversalMotorComponent::
expose_wires(WireContainer &out_r_wire_container)
{
  unsigned retval = out_r_wire_container.size();

  out_r_wire_container.insert(out_r_wire_container.end(), &wire);
  //out_r_wire_container.insert(&wire, out_r_wire_container.end());
  return retval;
}


void UniversalMotorComponent::draw() const
{
  // draws the axis...
  dVector3 v3_anchor;
  dVector3 v3_direction;


  float a_point_0[3];
  float a_point_1[3];

  dJointGetUniversalAnchor(joint_id, v3_anchor);


  dJointGetUniversalAxis1 (joint_id, v3_direction);
  
  for(unsigned i = 0; i < 3; ++i) {
    a_point_0[i] = v3_anchor[i] - v3_direction[i] / 2.0;
    a_point_1[i] = v3_anchor[i] + v3_direction[i] / 2.0;
  }

  dsDrawLine(a_point_0, a_point_1);

  dJointGetUniversalAxis2  (joint_id, v3_direction);
  
  for(unsigned i = 0; i < 3; ++i) {
    a_point_0[i] = v3_anchor[i] - v3_direction[i] / 2.0;
    a_point_1[i] = v3_anchor[i] + v3_direction[i] / 2.0;
  }

  dsDrawLine(a_point_0, a_point_1);
}


const IComponent* UniversalMotorComponent::
does_contain_geom(const dGeomID geom_id, bool b_recursive) const
{
  return NULL;
}

bool UniversalMotorComponent::
collision_callback(World *p_world, dGeomID geom_id_0, dGeomID geom_id_1) const
{
  return false;
}


unsigned UniversalMotorComponent::get_sub_component_count() const
{
  return 0;
}


IComponent& UniversalMotorComponent::get_sub_component(unsigned index) const
{
  exception::IndexOutOfBounds().raise();
}


/*****************************************************************************/
/* UniversalMotorWire                                                        */
/*****************************************************************************/
MotorWire::MotorWire(AbstractMotorComponent *_p_motor) :
  p_motor(_p_motor)
{
  if(NULL == p_motor)
    exception::InvalidArgument().raise();
}


IComponent& MotorWire::get_component() const
{
  return *p_motor;
}


dReal MotorWire::get() const
{
  return p_motor->get_angular_velocity();
}


void MotorWire::put(dReal value)
{
  p_motor->set_angular_velocity(value);
}



/*****************************************************************************/
/* SimplePhysicalComponent                                                   */
/*****************************************************************************/
SimplePhysicalComponent::
SimplePhysicalComponent(dBodyID _body_id, dGeomID _geom_id) :
  body_id(_body_id),
  geom_id(_geom_id)
{
}


void SimplePhysicalComponent::set_body_id(dBodyID _body_id)
{
  body_id = _body_id;
}


dBodyID SimplePhysicalComponent::get_body_id() const
{
  return body_id;
}


void SimplePhysicalComponent::set_geom_id(dGeomID _geom_id)
{
  geom_id = _geom_id;
}


dGeomID SimplePhysicalComponent::get_geom_id() const
{
  return geom_id;
}


unsigned SimplePhysicalComponent::
expose_wires(WireContainer &out_r_wire_container)
{
  return 0;
}


unsigned SimplePhysicalComponent::get_sub_component_count() const
{
  return 0;
}


IComponent& SimplePhysicalComponent::
get_sub_component(unsigned index) const
{
  exception::IndexOutOfBounds().raise();
}


// <TODO> this is not correct right now
// the snake component currently does all collision handling
// even though those simple components might define their own collission
// handling routine too
bool SimplePhysicalComponent::
collision_callback(World *p_world, dGeomID geom_id_0, dGeomID geom_id_1) const
{
  return false;
}

const IComponent* SimplePhysicalComponent::
does_contain_geom(const dGeomID _geom_id, bool b_recursive) const
{
  if(_geom_id == geom_id)
    return this;
  else return NULL;
}


void SimplePhysicalComponent::draw() const
{
  if(dCCylinderClass == dGeomGetClass(geom_id)) {
    dReal radius;
    dReal length;

    dGeomCCylinderGetParams(geom_id, &radius, &length);

    dsDrawCappedCylinder(dGeomGetPosition(geom_id), 
			 dGeomGetRotation(geom_id), 
			 length,
			 radius);

    /*    std::cout << length << "\n";
    std::cout << radius << "\n";
    std::cout << "---------------------\n";*/

  }
  else if(dBoxClass == dGeomGetClass(geom_id)) {
    dVector3 v3_length;
    dGeomBoxGetLengths(geom_id, v3_length);
    /*
    std::cout << "x = " << v3_length[0] << "\n";
    std::cout << "y = " << v3_length[1] << "\n";
    std::cout << "z = " << v3_length[2] << "\n";
    */
    dsDrawBox(dGeomGetPosition(geom_id),
	      dGeomGetRotation(geom_id),
	      v3_length);
  }
  else if(dPlaneClass == dGeomGetClass(geom_id)) {
    /*    dVector4 v4_params;
    dGeomPlaneGetParams(geom_id, v4_params);

    dsDrawPlane(dGeomGetPosition(geom_id),
		dGeomGetRotation(geom_id),
		v4[4]);    */
    // can't draw - there is no dsDrawPlane
  }
  else {

    std::cerr << "unknown geom type - cannot draw :(\n";
    //exception::
  }
  

}



/*****************************************************************************/
/* RobotArmComponent                                                         */
/*****************************************************************************/
/*
double atan_ex(double x, double y)
{
  if(0.0 == y)
    y = 0.00001;
  //return atan(x / y);

  if(0.0 <= x)  // 1, 4
    return atan(x / y);
  else {
    if(0.0 <= y)
      return PI + atan(x / y);
    else
      return -PI + atan(x / y);
   }
}
*/

/**
 * atan_ex
 *
 * utility function, not used here - should be moved to another file
 * 
 */
double atan_ex(double x, double y)
{

  if(0.0 == y)
    y = 0.00001; // replace by some epsilon constant

  if(0.0 <= x) {  // 1, 4
    if(0.0 <= y)
      return atan(x / y);
    else
      return 2 * PI + atan(x / y);
  }
  else
    return PI + atan(x / y);
}


/**
 * multiply
 *
 * is there some ode-multiply routine???!!!
 */
void multiply(dVector3 v3_result, const dMatrix3 m33, const dVector3 v3)
{
  v3_result[0] = m33[0 * 3 + 0] * v3[0] +
                 m33[0 * 3 + 1] * v3[1] + 
                 m33[0 * 3 + 2] * v3[2];

  v3_result[1] = m33[1 * 3 + 0] * v3[0] +
                 m33[1 * 3 + 1] * v3[1] + 
                 m33[1 * 3 + 2] * v3[2];

  v3_result[2] = m33[2 * 3 + 0] * v3[0] +
                 m33[2 * 3 + 1] * v3[1] + 
                 m33[2 * 3 + 2] * v3[2];
}


/**
 *
 *
 * this class class has following sub components:
 * UniversalMotorComponent
 * SimpleBodyComponent
 */
CCURobotArmComponent::CCURobotArmComponent(const RobotArmDescription &r_desc)
{
  if(NULL == r_desc.p_world)
    exception::InvalidArgument().raise();

  if(NULL == r_desc.p_vertex_list)
    exception::InvalidArgument().raise();

  if(1 >= r_desc.p_vertex_list->size())
    exception::InvalidArgument().raise();

  std::cout << "here\n";


  p_world = r_desc.p_world;

  /*  if(NULL == r_desc.space_id)
      space_id = dCreateHashSpace(0); */


  /*  segment_list.resize(r_desc.segment_count);
   */
  joint_group_id = dJointGroupCreate(0);

  dMass mass;




  
  VertexList::const_iterator it_vertex_previous =
    r_desc.p_vertex_list->begin();

  VertexList::const_iterator it_vertex_current =
    ++r_desc.p_vertex_list->begin();



  dBodyID body_id_previous;
  dBodyID body_id_current;

  dGeomID geom_id_current;
  Vector3<dReal> v3_delta;
  
  // rotation stuff
  Vector3<dReal> v3_rotation_axis;
    
  dQuaternion a_q_rotation[2];
    
  dQuaternion *p_q_rotation_previous = &a_q_rotation[0];
  dQuaternion *p_q_rotation_current  = &a_q_rotation[1];
    
  while(r_desc.p_vertex_list->end() != it_vertex_current) {
 
    v3_delta = *it_vertex_current - *it_vertex_previous;



    body_id_current = dBodyCreate(p_world->get_world_id());


    dMassSetCappedCylinderTotal(&mass, 
				r_desc.segment_mass,
				2,
				r_desc.segment_radius,
				v3_delta.length());


    dBodySetMass(body_id_current, &mass);


    geom_id_current = dCreateCCylinder(p_world->get_space_id(), 
				       r_desc.segment_radius, 
				       v3_delta.length());

    dGeomSetBody(geom_id_current, body_id_current);




    // rotation_axis = angle symmetry axis
    v3_rotation_axis = (v3_delta.make_unit_length() - 
			Vector3<dReal>(0.0, 0.0, 1.0)) / 2.0;

    v3_rotation_axis.make_unit_length();

    
    
    dQFromAxisAndAngle(*p_q_rotation_current,
		       v3_rotation_axis.x,
		       v3_rotation_axis.y,
		       v3_rotation_axis.z,
		       PI);
    
    dGeomSetQuaternion(geom_id_current, *p_q_rotation_current);
    

    // the position of the body is exactly between 2 control points
    dGeomSetPosition(geom_id_current,
    r_desc.v3_position.x + (it_vertex_previous->x + it_vertex_current->x) / 2,
    r_desc.v3_position.y + (it_vertex_previous->y + it_vertex_current->y) / 2,
    r_desc.v3_position.z + (it_vertex_previous->z + it_vertex_current->z) / 2);
    

    component_container.insert(component_container.end(),
			       new SimplePhysicalComponent(body_id_current,
							   geom_id_current));





    if(it_vertex_previous != r_desc.p_vertex_list->begin()) {
      
      dJointID joint_id_current;
      // set up the joints
      // since i does not equal zero, new and old rotation matrices are valid


      joint_id_current = dJointCreateUniversal(p_world->get_world_id(), 
					       joint_group_id);

      dJointAttach(joint_id_current, body_id_previous, body_id_current);

      // set the anchor
      dJointSetUniversalAnchor(joint_id_current,
			       r_desc.v3_position.x + it_vertex_previous->x,
			       r_desc.v3_position.x + it_vertex_previous->y,
			       r_desc.v3_position.x + it_vertex_previous->z);

      // so... lets try to rotate the hinge axis...

      // set axis 1 (belongs to the previous segment)
      {
	dVector3 v3_axis = {0.0, 1.0, 0.0};
	dVector3 v3_rotated_axis;
	dMatrix3 m;


	dQtoR(*p_q_rotation_previous, m);

	multiply(v3_rotated_axis, m, v3_axis);

	dJointSetUniversalAxis1(joint_id_current,
				v3_rotated_axis[0],
				v3_rotated_axis[1],
				v3_rotated_axis[2]);

	component_container.
	  insert(component_container.end(),
		 new UniversalMotorComponent(joint_id_current,
					     0));
	
	// beginning of temporary section
	{	  
	  /*
	  dBodyID b_tmp = dBodyCreate(p_world->get_world_id());
	  dGeomID g_tmp = dCreateBox (p_world->get_space_id(), 
				      0.05, 
				      1.0,
				      0.05);


	  dGeomSetQuaternion(g_tmp, *p_q_rotation_previous);    

	  
	  dGeomSetPosition(g_tmp,
			   r_desc.v3_position.x + it_vertex_previous->x,
			   r_desc.v3_position.x + it_vertex_previous->y,
			   r_desc.v3_position.x + it_vertex_previous->z);

	  component_container.insert(component_container.end(),
				     new SimplePhysicalComponent(b_tmp,
								 g_tmp));
	  */
      
	}
	// end of temporary section
	
      }


      // set axis 2 (belongs to the previous segment)
      {
	dVector3 v3_axis = {1.0, 0.0, 0.0};
	dVector3 v3_rotated_axis;
	dMatrix3 m;

	dQtoR(*p_q_rotation_current, m);

	multiply(v3_rotated_axis, m, v3_axis);

	dJointSetUniversalAxis2(joint_id_current,
				v3_rotated_axis[0],
				v3_rotated_axis[1],
				v3_rotated_axis[2]);


	component_container.
	  insert(component_container.end(),
		 new UniversalMotorComponent(joint_id_current,
					     1));

	
	// beginning of temporary section
	{
	  /*
	  dBodyID b_tmp = dBodyCreate(p_world->get_world_id());
	  dGeomID g_tmp = dCreateBox (p_world->get_space_id(), 
				      1.0, 
				      0.05,
				      0.05);


	  dGeomSetQuaternion(g_tmp, *p_q_rotation_current);    

	  
	  dGeomSetPosition(g_tmp,
			   r_desc.v3_position.x + it_vertex_previous->x,
			   r_desc.v3_position.x + it_vertex_previous->y,
			   r_desc.v3_position.x + it_vertex_previous->z);

	  component_container.insert(component_container.end(),
				     new SimplePhysicalComponent(b_tmp,
								 g_tmp));
	  */
	}
	// end of temporary section
	
      }


    }


 
    std::swap(p_q_rotation_previous, p_q_rotation_current);
    std::swap(body_id_previous     , body_id_current);
 

    ++it_vertex_previous;
    ++it_vertex_current;
  }

}



CCURobotArmComponent::~CCURobotArmComponent()
{
  // delete all components
  ComponentContainer::iterator it = component_container.begin();
  while(component_container.end() != it) {
    delete *it;
    ++it;
  }

  dJointGroupDestroy(joint_group_id);
}


void CCURobotArmComponent::draw() const
{
  // for all sub components: draw()
  ComponentContainer::const_iterator it = component_container.begin();
  while(component_container.end() != it) {
    (*it)->draw();
    ++it;
  }
}


unsigned CCURobotArmComponent::get_sub_component_count() const
{
  return component_container.size();
}


IComponent &CCURobotArmComponent::get_sub_component(unsigned index) const
{
  if(index >= component_container.size())
    exception::IndexOutOfBounds().raise();

  ComponentContainer::const_iterator it = component_container.begin();
  for(unsigned i = 0; i < index; ++i)
    ++it;

  return *(*it);;
}


unsigned CCURobotArmComponent::
expose_wires(WireContainer &out_r_wire_container)
{
  unsigned retval = out_r_wire_container.size();

  ComponentContainer::const_iterator it = component_container.begin();

  for(it; component_container.end() != it; ++it)
    (*it)->expose_wires(out_r_wire_container);

  return retval;
}


const IComponent* CCURobotArmComponent::
does_contain_geom(dGeomID geom_id, bool b_recursive) const
{
  const IComponent *p;
  ComponentContainer::const_iterator it = component_container.begin();

  for(it; component_container.end() != it; ++it) {
    p = (*it)->does_contain_geom(geom_id, b_recursive);

    if(NULL != p)
      return p;
  }
  

  return NULL;
}


/**
 * collision_callback
 *
 * this function only does collision handling for geoms part of this object
 *
 * hmm.. there should be a mechanism for transferring collision handling
 * to sub components
 */
bool CCURobotArmComponent::collision_callback(World *p_world, 
					     dGeomID geom_id_0, 
					     dGeomID geom_id_1) const
{
  //World *p_world = reinterpret_cast<World*>(p_data);

  // check if at least one of the two geoms is part of this component
  const IComponent *p_component_0 = does_contain_geom(geom_id_0, true);
  const IComponent *p_component_1 = does_contain_geom(geom_id_1, true);

  if(NULL == p_component_0 && NULL == p_component_1)
    return false;

  // at least one geom is part of one of this component's (sub) components

  // check if it is a collision between two adjacing segments
  // these collisions will be ignored
  

  dBodyID body_id_0 = dGeomGetBody(geom_id_0);
  dBodyID body_id_1 = dGeomGetBody(geom_id_1);


  if(NULL != body_id_0 && NULL != body_id_1) {
    // if those bodies are connected: no collision
    if(dAreConnected(body_id_0, body_id_1))
      return false;
  }


  /*
  JointIdList::iterator it = lst_joint_id.begin();
  while(lst_joint_id.end() != it) {
    if((dJointGetBody(*it, 0) == body_id_0 && 
      dJointGetBody(*it, 1) == body_id_1) || 
      (dJointGetBody(*it, 1) == body_id_0 && 
      dJointGetBody(*it, 0) == body_id_1))
    return false;
    }
  */

  // ok, we have a collision here - create the contact joints
  const unsigned max_contact_count = 10;
  unsigned n;
  dContact a_contact[max_contact_count];

  n = dCollide(geom_id_0,
	       geom_id_1,
	       max_contact_count,
	       &a_contact->geom,
	       sizeof(dContact));

  for (unsigned i = 0; i < n; ++i) {
    a_contact[i].surface.mode = dContactSlip1   | dContactSlip2   |
                                dContactSoftERP | dContactSoftCFM | 
                                dContactApprox1;
    a_contact[i].surface.mu       = 0.8;
    a_contact[i].surface.slip1    = 0.005;
    a_contact[i].surface.slip2    = 0.005;
    a_contact[i].surface.soft_erp = 1;
    a_contact[i].surface.soft_cfm = 0.00001;
    
    dJointID joint_id = 
      dJointCreateContact(p_world->get_world_id(), 
			  p_world->get_joint_group_id_contact(), 
			  &a_contact[i]);

    dJointAttach(joint_id ,
		 dGeomGetBody(a_contact[i].geom.g1), 
		 dGeomGetBody(a_contact[i].geom.g2));
  }


  // !!! maybe the world should be a container for robots 
  // (instead of components)
  // the following line causes problems when components are used recursively
  // (and recursive usage is the main purpose of components)
  //p_world->register_component(this);

  return true;
}



/*****************************************************************************/
/* PlaygroundComponent                                                       */
/*****************************************************************************/
/*PlaygroundComponent::PlaygroundComponent(const PlaygroundDescription &r_desc)
{
  
}*/


/*****************************************************************************/
/* PlaneComponent                                                            */
/*****************************************************************************/
PlaneComponentDescription::PlaneComponentDescription()
{
  v3_normal = Vector3<dReal>(0.0, 0.0, 1.0);
  d         = 0.0;
}


PlaneComponent::PlaneComponent(const PlaneComponentDescription &r_desc)
{
  if(NULL == r_desc.p_world)
    exception::InvalidArgument().raise();

  geom_id = dCreatePlane(r_desc.p_world->get_space_id(),
			 r_desc.v3_normal.x,
			 r_desc.v3_normal.y,
			 r_desc.v3_normal.z,
			 r_desc.d);
}


PlaneComponent::~PlaneComponent()
{
  dGeomDestroy(geom_id);
}





}
}

