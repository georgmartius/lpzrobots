/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 *   $Log$
 *   Revision 1.6  2005-11-09 13:27:32  fhesse
 *   GPL added
 *                                                                * 
/***************************************************************************/

#include "component.h"
#include "drawgeom.h"

namespace university_of_leipzig {
namespace robots {

  OdeHandle temp(NULL, NULL, NULL);
  //  dJointGroupID joint_group_id = dJointGroupCreate(0);

/*****************************************************************************/
/* AbstractComponent                                                         */
/*****************************************************************************/

AbstractComponent::AbstractComponent(OdeHandle &r_ode_handle) :
  ode_handle(r_ode_handle)
{
}


AbstractComponent::~AbstractComponent()
{
}


unsigned AbstractComponent::get_sub_component_count() const
{
  return 0;
}


IComponent& AbstractComponent::get_sub_component(unsigned index) const
{
  IndexOutOfBoundsException().raise();
 
  OdeHandle oh(NULL, NULL, NULL);
  return *(new SimplePhysicalComponent(oh, NULL, NULL));
}


unsigned AbstractComponent::
expose_wires(WireContainer &out_r_wire_container)
{
  return 0;
}


const IComponent* AbstractComponent::
does_contain_geom(const dGeomID geom_id, bool b_recursive) const
{
  return NULL;
}


Configurable::paramkey AbstractComponent::getName() const
{
  return static_cast<paramkey>("SomeComponent");
}


Configurable::paramlist AbstractComponent::getParamList() const
{
  paramlist list;

  return list;
}


Configurable::paramval AbstractComponent::
getParam(const paramkey& key) const
{
  return Configurable::getParam(key) ;
}


bool AbstractComponent::setParam(const paramkey& key, paramval val){
  return Configurable::setParam(key, val);
}


/*****************************************************************************/
/* AbstractCompoundComponent                                                 */
/*****************************************************************************/

AbstractCompoundComponent::
AbstractCompoundComponent(OdeHandle &r_ode_handle) :
  AbstractComponent(r_ode_handle)
{
}


AbstractCompoundComponent::~AbstractCompoundComponent()
{
}


unsigned AbstractCompoundComponent::get_sub_component_count() const
{
  return component_container.size();
}


IComponent &AbstractCompoundComponent::get_sub_component(unsigned index) const
{
  if(index >= component_container.size())
    IndexOutOfBoundsException().raise();

  ComponentContainer::const_iterator it = component_container.begin();
  for(unsigned i = 0; i < index; ++i)
    ++it;

  return *(*it);;
}


unsigned AbstractCompoundComponent::
expose_wires(WireContainer &out_r_wire_container)
{
  unsigned retval = 0; //out_r_wire_container.size();

  ComponentContainer::const_iterator it = component_container.begin();

  while(component_container.end() != it)
    retval += (*(it++))->expose_wires(out_r_wire_container);

  return retval;
}


const IComponent* AbstractCompoundComponent::
does_contain_geom(dGeomID geom_id, bool b_recursive) const
{
  const IComponent *p;
  ComponentContainer::const_iterator it = component_container.begin();

  while(component_container.end() != it) {
    p = (*(it++))->does_contain_geom(geom_id, b_recursive);

    if(NULL != p)
      return p;
  }
  

  return NULL;
}


void AbstractCompoundComponent::draw() const
{
  // for all sub components: draw()
  ComponentContainer::const_iterator it = component_container.begin();
  while(component_container.end() != it) {
    (*it)->draw();
    ++it;
  }
}


Configurable::paramlist AbstractCompoundComponent::getParamList() const
{
  paramlist list;

  for(ComponentContainer::const_iterator it = component_container.begin();
      it != component_container.end();
      ++it) {
    list += (*it)->getParamList();
  }

  return list;
}


Configurable::paramval AbstractCompoundComponent::
getParam(const paramkey& key) const
{
  paramval val = 0.0;
  ComponentContainer::const_iterator it = component_container.begin();

  while(it != component_container.end() && val == 0.0) {
    val = (*it)->getParam(key);    
    ++it;
  }

  return val;
}


bool AbstractCompoundComponent::setParam(const paramkey& key, paramval val)
{
  for(ComponentContainer::iterator it = component_container.begin();
      it != component_container.end();
      ++it)
    (*it)->setParam(key, val);

  return true;
}



/*****************************************************************************/
/* AbstractMotorComponent                                                    */
/*****************************************************************************/
AbstractMotorComponent::AbstractMotorComponent(dJointID _joint_id) :
  AbstractComponent(temp),
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
    InvalidArgumentException().raise();

  if(axis != 0 && axis != 1)
    InvalidArgumentException().raise();

  param_show_axis = 1.0;
}


IComponent::paramlist UniversalMotorComponent::getParamList() const
{
  paramlist list;
  list.push_back(pair<paramkey, paramval> (string("show_axis"), 
					   param_show_axis));

  return list;
}


Configurable::paramval UniversalMotorComponent::
getParam(const paramkey& key) const
{
  if(key == "show_axis")
    return param_show_axis; 
  else
    return Configurable::getParam(key) ;
}


bool UniversalMotorComponent::setParam(const paramkey& key, paramval val){
  if(key == "show_axis")
    param_show_axis = val;
  else
    return Configurable::setParam(key, val);

  return true;
}



void UniversalMotorComponent::
set_angular_velocity(dReal angular_velocity)
{
  //  std::cout << angular_velocity << "\n";
  if(0 == axis) {
    dJointSetUniversalParam (joint_id ,dParamVel,  angular_velocity * 10);
    dJointSetUniversalParam (joint_id, dParamFMax, 2.5);		
  }
  else if(1 == axis) {
    dJointSetUniversalParam (joint_id ,dParamVel2, angular_velocity * 10);
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

  return static_cast<dReal>(0);
}


unsigned UniversalMotorComponent::
expose_wires(WireContainer &out_r_wire_container)
{
  //d  unsigned retval = out_r_wire_container.size();

  out_r_wire_container.insert(out_r_wire_container.end(), &wire);
  //out_r_wire_container.insert(&wire, out_r_wire_container.end());
  return 1;
}


void UniversalMotorComponent::draw() const
{
  // draws the axis...
  dVector3 v3_anchor;
  //dVector3 v3_anchor2;

  dVector3 v3_direction;

  float a_point_0[3];
  float a_point_1[3];

  dJointGetUniversalAnchor(joint_id, v3_anchor);

  // dJointGetUniversalAnchor2(joint_id, v3_anchor2);



  if(param_show_axis >= 0.5) {
    if(0 == axis) {
      dJointGetUniversalAxis1 (joint_id, v3_direction);
      
      for(unsigned i = 0; i < 3; ++i) {
	a_point_0[i] = v3_anchor[i] - v3_direction[i] / 2.0;
	a_point_1[i] = v3_anchor[i] + v3_direction[i] / 2.0;
      }

      dsSetColor(1.0, 0.0, 0.0);
      dsDrawLine(a_point_0, a_point_1);
      /*
      for(unsigned j = 0; j < 3; ++j) {
	a_point_0[j] = v3_anchor2[j] - v3_direction[j] / 2.0;
	a_point_1[j] = v3_anchor2[j] + v3_direction[j] / 2.0;
      }

      dsSetColor(1.0, 0.0, 0.0);
      dsDrawLine(a_point_0, a_point_1);
      */
    }
    else { // ( 1 == axis)

      dJointGetUniversalAxis2  (joint_id, v3_direction);
  
      for(unsigned i = 0; i < 3; ++i) {
	a_point_0[i] = v3_anchor[i] - v3_direction[i] / 2.0;
	a_point_1[i] = v3_anchor[i] + v3_direction[i] / 2.0;
      }

      dsSetColor(0.0, 0.0, 1.0);
      dsDrawLine(a_point_0, a_point_1);


      /*
      for(unsigned j = 0; j < 3; ++j) {
	a_point_0[j] = v3_anchor2[j] - v3_direction[j] / 2.0;
	a_point_1[j] = v3_anchor2[j] + v3_direction[j] / 2.0;
      }

      dsSetColor(1.0, 0.0, 0.0);
      dsDrawLine(a_point_0, a_point_1);
      */

    }
  }
  dsSetColor(1.0, 1.0, 1.0);
}

/*
const IComponent* UniversalMotorComponent::
does_contain_geom(const dGeomID geom_id, bool b_recursive) const
{
  return NULL;
}
*/

bool UniversalMotorComponent::
collision_callback(OdeHandle *p_ode_handle, dGeomID geom_id_0, dGeomID geom_id_1) const
{
  return false;
}

/*
unsigned UniversalMotorComponent::get_sub_component_count() const
{
  return 0;
}


IComponent& UniversalMotorComponent::get_sub_component(unsigned index) const
{
  IndexOutOfBoundsException().raise();

  return *(new SimplePhysicalComponent());
}
*/

/*****************************************************************************/
/* UniversalMotorWire                                                        */
/*****************************************************************************/
MotorWire::MotorWire(AbstractMotorComponent *_p_motor) :
  p_motor(_p_motor)
{
  if(NULL == p_motor)
    InvalidArgumentException().raise();
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
SimplePhysicalComponent(OdeHandle &r_ode_handle,
			dBodyID _body_id,
			dGeomID _geom_id) :
  AbstractComponent(r_ode_handle),
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

/*
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
  IndexOutOfBoundsException().raise();

  return *(new SimplePhysicalComponent());
}
*/

// <TODO> this is not correct right now
// the snake component currently does all collision handling
// even though those simple components might define their own collission
// handling routine too
bool SimplePhysicalComponent::
collision_callback(OdeHandle *p_ode_handle, dGeomID geom_id_0, dGeomID geom_id_1) const
{
  return false;
  if(!(does_contain_geom(geom_id_0, true) ||
       does_contain_geom(geom_id_1, true)))
    return false;

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
    a_contact[i].surface.mu       = 0.8; //conf.frictionGround;
    a_contact[i].surface.slip1    = 0.005;
    a_contact[i].surface.slip2    = 0.005;
    a_contact[i].surface.soft_erp = 0.9;
    a_contact[i].surface.soft_cfm = 0.001;
    
    dJointID joint_id = 
      dJointCreateContact(ode_handle.world, 
			  ode_handle.jointGroup,
			  &a_contact[i]);

    dJointAttach(joint_id ,
		 dGeomGetBody(a_contact[i].geom.g1), 
		 dGeomGetBody(a_contact[i].geom.g2));
  }
  return true;
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
  drawGeom(geom_id, NULL, NULL);
  return;

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
  else if(dSphereClass == dGeomGetClass(geom_id)) {
    dsDrawSphere(dGeomGetPosition(geom_id), 
		 dGeomGetRotation(geom_id),
		 dGeomSphereGetRadius(geom_id));
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
      return 2 * M_PI + atan(x / y);
  }
  else
    return M_PI + atan(x / y);
}


double quat_mag(const dQuaternion q)
{
  double square_mag = 0.0;
  for(int i = 0; i < 4; ++i)
    square_mag += q[i] * q[i];

  return sqrt(square_mag);
}


void quat_norm(dQuaternion q)
{
  double mag = quat_mag(q);

  for(int i = 0; i < 4; ++i)
    q[i] /= mag;
}



/**
 * conjugate_quaternion
 *
 *
 */
void quat_conj(dQuaternion q)
{
  for(int i = 0; i < 3; ++i)
    q[i] = -q[i];
}


void quat_inv(dQuaternion q)
{
  dQuaternion tmp;

  memcpy(tmp ,q, sizeof(dQuaternion));
  quat_norm(tmp);
  quat_conj(tmp);

  memcpy(q ,tmp, sizeof(dQuaternion));
}


void qv_mult(dVector3 v3_result, dQuaternion q, const dVector3 v3)
{
  dQuaternion inv_q;

  memcpy(inv_q, q, sizeof(dQuaternion));

  quat_inv(inv_q);

  for(int i = 0; i < 4; ++i)
    v3_result[i] = q[i] * v3[i] * inv_q[i];
}



/**
 * multiply
 *
 * is there some ode-multiply routine???!!!
 */
void xxx_multiply(dVector3 v3_result, const dMatrix3 m33, const dVector3 v3)
{
  //  if(v3_result == v3)
    

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

void yyy_multiply(dVector3 v3_result, const dMatrix3 m33, const dVector3 v3)
{
  //  if(v3_result == v3)
    

  v3_result[0] = m33[0 * 4 + 0] * v3[0] +
                 m33[1 * 4 + 0] * v3[1] + 
                 m33[2 * 4 + 0] * v3[2];
    //m33[3 * 3 + 0];

  v3_result[1] = m33[0 * 4 + 1] * v3[0] +
                 m33[1 * 4 + 1] * v3[1] + 
                 m33[2 * 4 + 1] * v3[2];
    // m33[3 * 3 + 1];

  v3_result[2] = m33[0 * 4 + 2] * v3[0] +
                 m33[1 * 4 + 2] * v3[1] + 
                 m33[2 * 4 + 2] * v3[2];
    // m33[3 * 3 + 2];
}

/**
 *
 *
 * this class class has following sub components:
 * UniversalMotorComponent
 * SimpleBodyComponent
 */
CCURobotArmComponent::CCURobotArmComponent(const RobotArmDescription &r_desc) :
  // ode_handle(*(r_desc.p_ode_handle))
  AbstractCompoundComponent(*r_desc.p_ode_handle)
{
  if(NULL == r_desc.p_ode_handle)
    InvalidArgumentException().raise();

  if(NULL == r_desc.p_vertex_list)
    InvalidArgumentException().raise();

  if(1 >= r_desc.p_vertex_list->size())
    InvalidArgumentException().raise();

  //  ode_handle = *r_desc.p_ode_handle;

  joint_group_id = dJointGroupCreate(0);

  dMass mass;

  
  VertexList::const_iterator it_vertex_previous =
    r_desc.p_vertex_list->begin();

  VertexList::const_iterator it_vertex_current =
    ++r_desc.p_vertex_list->begin();


  dBodyID body_id_previous = NULL;
  dBodyID body_id_current  = NULL;

  dGeomID geom_id_current;
  Vector3<dReal> v3_delta;
  
  // rotation stuff
  Vector3<dReal> v3_rotation_axis;
    
  dQuaternion a_q_rotation[2];
    
  dQuaternion *p_q_rotation_previous = &a_q_rotation[0];
  dQuaternion *p_q_rotation_current  = &a_q_rotation[1];
 
  std::cout << "size = " << r_desc.p_vertex_list->size() << "\n";
  while(r_desc.p_vertex_list->end() != it_vertex_current) {
 
    std::cout << "ANOTHER LOOP\n";
    v3_delta = *it_vertex_current - *it_vertex_previous;



    body_id_current = dBodyCreate(ode_handle.world);


    /*
    dMassSetCappedCylinderTotal(&mass, 
				r_desc.segment_mass,
				2,
				r_desc.segment_radius,
				v3_delta.length());

    */

    dMassSetCappedCylinder(&mass,
			   1,
			   1,
			   r_desc.segment_radius,
			   v3_delta.length());

    dMassAdjust(&mass, r_desc.segment_mass);

    dBodySetMass(body_id_current, &mass);


    geom_id_current = dCreateCCylinder(ode_handle.space, 
				       r_desc.segment_radius, 
				       v3_delta.length());

 
    // rotation_axis = angle symmetry axis
    v3_rotation_axis = Vector3<dReal>(0.0, 0.0, 1.0) +
                       (v3_delta.get_unit_vector() - 
                        Vector3<dReal>(0.0, 0.0, 1.0)) / 2.0;
  
    v3_rotation_axis.make_unit_length();

    
    dQFromAxisAndAngle(*p_q_rotation_current,
		       v3_rotation_axis.x,
		       v3_rotation_axis.y,
		       v3_rotation_axis.z,
		       M_PI);

    dGeomSetBody(geom_id_current, body_id_current);

    dBodySetPosition(body_id_current,
    r_desc.v3_position.x + (it_vertex_previous->x + it_vertex_current->x)/2.0,
    r_desc.v3_position.y + (it_vertex_previous->y + it_vertex_current->y)/2.0,
    r_desc.v3_position.z + (it_vertex_previous->z + it_vertex_current->z)/2.0);

    dBodySetQuaternion(body_id_current, *p_q_rotation_current);
    

    // the position of the body is exactly between 2 control points
    //dBodySetPosition(geom_id_current,


    component_container.insert(component_container.end(),
			       new SimplePhysicalComponent(ode_handle,
							   body_id_current,
							   geom_id_current));



    if(it_vertex_previous != r_desc.p_vertex_list->begin()) {
      std::cout << "sdfsdfhdsovbgfdshzvfbsd\n";
      dJointID joint_id_current;
      // set up the joints
      // since i does not equal zero, new and old rotation matrices are valid


      joint_id_current = dJointCreateUniversal(ode_handle.world,
					       0);

      dJointAttach(joint_id_current, body_id_previous, body_id_current);

      // set the anchor
      dJointSetUniversalAnchor(joint_id_current,
			       r_desc.v3_position.x + it_vertex_previous->x,
			       r_desc.v3_position.x + it_vertex_previous->y,
			       r_desc.v3_position.x + it_vertex_previous->z);

      // so... lets try to rotate the hinge axis...

      // set axis 1 (belongs to the previous segment)
      {
	dVector3 a_point_0 = {-0.5, 0.0, v3_delta.length() / 2.0, 0.0};
	dVector3 a_point_1 = { 0.5, 0.0, v3_delta.length() / 2.0, 0.0};
	dVector3 a_rotated_point_0;
	dVector3 a_rotated_point_1;
	dVector3 a_rotated_axis;
	dMatrix3 a_m;


	dQtoR(*p_q_rotation_previous, a_m);

	yyy_multiply(a_rotated_point_0, a_m, a_point_0);
	yyy_multiply(a_rotated_point_1, a_m, a_point_1);



	for(int i = 0; i < 3; ++i)
	  a_rotated_axis[i] = a_rotated_point_1[i] - a_rotated_point_0[i];


	dJointSetUniversalAxis1(joint_id_current,
				a_rotated_axis[0],
				a_rotated_axis[1],
				a_rotated_axis[2]);

	component_container.
	  insert(component_container.end(),
		 new UniversalMotorComponent(joint_id_current,
					     0));	
      }


      // set axis 2 (belongs to the current segment)
      {
	dVector3 a_point_0 = {0.0, -0.5, v3_delta.length() / 2.0, 0.0};
	dVector3 a_point_1 = {0.0,  0.5, v3_delta.length() / 2.0, 0.0};
	dVector3 a_rotated_point_0;
	dVector3 a_rotated_point_1;
	dVector3 a_rotated_axis;
	dMatrix3 a_m;


	dQtoR(*p_q_rotation_current, a_m);

	yyy_multiply(a_rotated_point_0, a_m, a_point_0);
	yyy_multiply(a_rotated_point_1, a_m, a_point_1);

	for(int i = 0; i < 3; ++i)
	  a_rotated_axis[i] = a_rotated_point_1[i] - a_rotated_point_0[i];

	dJointSetUniversalAxis2(joint_id_current,
				a_rotated_axis[0],
				a_rotated_axis[1],
				a_rotated_axis[2]);

	component_container.
	  insert(component_container.end(),
		 new UniversalMotorComponent(joint_id_current,
					     1));
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

/*
unsigned CCURobotArmComponent::
expose_wires(WireContainer &out_r_wire_container)
{
  unsigned retval = out_r_wire_container.size();

  ComponentContainer::const_iterator it = component_container.begin();

  while(component_container.end() != it)
    (*(it++))->expose_wires(out_r_wire_container);

  return retval;
}
*/

const IComponent* CCURobotArmComponent::
does_contain_geom(dGeomID geom_id, bool b_recursive) const
{
  const IComponent *p;
  ComponentContainer::const_iterator it = component_container.begin();

  while(component_container.end() != it) {
    p = (*(it++))->does_contain_geom(geom_id, b_recursive);

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
bool CCURobotArmComponent::collision_callback(OdeHandle *p_ode_handle, 
					     dGeomID geom_id_0, 
					     dGeomID geom_id_1) const
{
  // check if at least one of the two geoms is part of this component
  const IComponent *p_component_0 = does_contain_geom(geom_id_0, true);
  const IComponent *p_component_1 = does_contain_geom(geom_id_1, true);

  // if && is used instead: both geoms must be part of this component
  if(NULL == p_component_0 || NULL == p_component_1)
    return false;

  // at least one geom is part of one of this component's (sub) components

  // check if it is a collision between two adjacing segments
  // these collisions will be ignored
  

  dBodyID body_id_0 = dGeomGetBody(geom_id_0);
  dBodyID body_id_1 = dGeomGetBody(geom_id_1);


  if(NULL != body_id_0 && NULL != body_id_1) {
    // if those bodies are connected: no collision
    if(dAreConnected(body_id_0, body_id_1))
      return true;
  }
  return false;

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
    a_contact[i].surface.mu       = 0.8; //conf.frictionGround;
    a_contact[i].surface.slip1    = 0.005;
    a_contact[i].surface.slip2    = 0.005;
    a_contact[i].surface.soft_erp = 0.9;
    a_contact[i].surface.soft_cfm = 0.00001;
    
    dJointID joint_id = 
      dJointCreateContact(ode_handle.world, 
			  ode_handle.jointGroup,
			  &a_contact[i]);

    dJointAttach(joint_id ,
		 dGeomGetBody(a_contact[i].geom.g1), 
		 dGeomGetBody(a_contact[i].geom.g2));
  }


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
/*PlaneComponentDescription::PlaneComponentDescription()
{
  v3_normal = Vector3<dReal>(0.0, 0.0, 1.0);
  d         = 0.0;
}


PlaneComponent::PlaneComponent(const PlaneComponentDescription &r_desc) :
  AbstractComponent(r_desc.p_ode_handle)
{
  if(NULL == r_desc.p_ode_handle)
    InvalidArgumentException().raise();

  geom_id = dCreatePlane(r_desc.p_ode_handle->space,
			 r_desc.v3_normal.x,
			 r_desc.v3_normal.y,
			 r_desc.v3_normal.z,
			 r_desc.d);
}


PlaneComponent::~PlaneComponent()
{
  dGeomDestroy(geom_id);
}

*/

SpiderDescription::SpiderDescription()
{

}


/**
 * SpiderComponent
 *
 * this class class has following sub components:
 * UniversalMotorComponent
 * CCURobotArmComponent
 * SimpleBodyComponent
 */
SpiderComponent::SpiderComponent(const SpiderDescription &r_desc) :
  AbstractCompoundComponent(*r_desc.p_ode_handle)
  //  ode_handle(*(r_desc.p_ode_handle))
{
  if(NULL == r_desc.p_ode_handle)
    InvalidArgumentException().raise();

  if(NULL == r_desc.p_angle_list)
    InvalidArgumentException().raise();

  if(0 >= r_desc.p_angle_list->size())
    InvalidArgumentException().raise();

  //  ode_handle = *r_desc.p_ode_handle;

  //joint_group_id = dJointGroupCreate(0);


  dMass   spider_body_mass;
  dBodyID spider_body_id;
  dGeomID spider_geom_id;



  // create the body (for the sphere)
  spider_body_id = dBodyCreate(ode_handle.world);


  // set up the mass
  dMassSetSphereTotal(&spider_body_mass, 
		      r_desc.sphere_mass,
		      r_desc.sphere_radius);

  dBodySetMass(spider_body_id, &spider_body_mass);


  // set up the sphere geom
  spider_geom_id = dCreateSphere(ode_handle.space, 
				 r_desc.sphere_radius);

  /*
  dMassSetBoxTotal(&spider_body_mass,
		   1.0,
		   1.0, 1.0, 0.25);

  dBodySetMass(spider_body_id, &spider_body_mass);

  spider_geom_id = dCreateBox(ode_handle.space, 
			      1.0, 1.0, 0.25);
  */
  /*
  std::cout << "-----------------" << "\n";
  std::cout << r_desc.position.x << "\n";
  std::cout << r_desc.position.y << "\n";
  std::cout << r_desc.position.z << "\n";
  */
  dGeomSetBody(spider_geom_id, spider_body_id);


  dGeomSetPosition(spider_geom_id,
		   r_desc.position.x,
		   r_desc.position.y,
		   r_desc.position.z);

  /*
  dGeomSdetPosition(spider_geom_id,
		   r_desc.position.x,
		   r_desc.position.y,
		   r_desc.position.z);
  */
  // insert a simple physical component into the component container
  component_container.insert(component_container.end(),
			     new SimplePhysicalComponent(ode_handle,
							 spider_body_id,
							 spider_geom_id));


  VertexList vl;

  // create the joints (which are needed for connecting the legs)
  // and the legs
  for(AngleList::iterator it_angle = r_desc.p_angle_list->begin();
      it_angle != r_desc.p_angle_list->end();
      ++it_angle) {


    // create the robot arm component
    RobotArmDescription rad;


    vl.clear();
    // position of the segments:
    // x = cos(angle) * (sphere_radius + segment_length * i)

    for(unsigned i = 0; i <= r_desc.segment_count; ++i) {
      dReal r = r_desc.sphere_radius +
	        r_desc.segment_radius + 0.1 + r_desc.segment_length * i;
      vl.insert(vl.end(), Vertex(cos(it_angle->x) * r + r_desc.position.x,
				 sin(it_angle->x) * r + r_desc.position.y,
				 0.0                  + r_desc.position.z));
    }
 
    rad.p_ode_handle   = r_desc.p_ode_handle;
    rad.segment_mass   = r_desc.segment_mass;
    rad.segment_radius = r_desc.segment_radius;
    rad.p_vertex_list  = &vl;
 
    std::cout << "CREATING\n";
    CCURobotArmComponent *p_robot_arm = new CCURobotArmComponent(rad);

    // insert the robot arm into the sub component container
    component_container.
      insert(component_container.end(),
	     p_robot_arm);


    // cast that subcomponent to a simple component
    SimplePhysicalComponent *p_segment =
    dynamic_cast<SimplePhysicalComponent*>(&p_robot_arm->get_sub_component(0));


    const dReal *vv;
    vv = dGeomGetPosition(p_segment->get_geom_id());

    std::cout << "...---..." << p_robot_arm->get_sub_component_count() << "\n";
    std::cout << vv[0] << "\n";
    std::cout << vv[1] << "\n";
    std::cout << vv[2] << "\n";

    // create a new joint
    dJointID joint_id = dJointCreateUniversal(ode_handle.world,
					      0);
	
    // attach the first segment of the arm to the body
    dJointAttach(joint_id, spider_body_id, p_segment->get_body_id());
    // dJointAttach(joint_id, spider_body_id,  p_segment->get_body_id());


    // set the anchor of the joint
    VertexList::iterator it_vertex = vl.begin();
    dJointSetUniversalAnchor(joint_id,
			     it_vertex->x, 
			     it_vertex->y,
			     it_vertex->z);
    
    /*
    std::cout << "----------------\n";
    std::cout << it_vertex->x << "\n";
    std::cout << it_vertex->y << "\n";
    std::cout << it_vertex->z << "\n";
    */
    // set up first axis
    dJointSetUniversalAxis1(joint_id, 0.0, 0.0, 1.0);


    // set up second axis
    dJointSetUniversalAxis2(joint_id,
			    sin(it_angle->x),
			    -cos(it_angle->x),
			    0.0);
    
    dVector3 v;
    dJointGetUniversalAnchor(joint_id, v);
    std::cout << "x1 = " << v[0] << "\n";
    std::cout << "y1 = " << v[1] << "\n";
    std::cout << "z1 = " << v[2] << "\n";
 
    dJointGetUniversalAnchor2(joint_id, v);
    std::cout << "x2 = " << v[0] << "\n";
    std::cout << "y2 = " << v[1] << "\n";
    std::cout << "z2 = " << v[2] << "\n";


    // set up first motor
    component_container.
      insert(component_container.end(),
	     new UniversalMotorComponent(joint_id, 0));

    // set up second motor
    component_container.
      insert(component_container.end(),
	     new UniversalMotorComponent(joint_id, 1));
    
  }
}



/**
 * collision_callback
 *
 * this function only does collision handling for geoms part of this object
 *
 * hmm.. there should be a mechanism for transferring collision handling
 * to sub components
 */
bool SpiderComponent::collision_callback(OdeHandle *p_ode_handle, 
					 dGeomID geom_id_0, 
					 dGeomID geom_id_1) const
{
  // check if the sphere collided with something (the first entry
  // in the component container)
  ComponentContainer::const_iterator it = component_container.begin(); 

  const SimplePhysicalComponent *p_spider_body =
    dynamic_cast<const SimplePhysicalComponent*>(*it);
  
  if(geom_id_0 == p_spider_body->get_geom_id() ||
     geom_id_1 == p_spider_body->get_geom_id()) {
    return false;
    /*
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
      a_contact[i].surface.mu       = 0.8; //conf.frictionGround;
      a_contact[i].surface.slip1    = 0.005;
      a_contact[i].surface.slip2    = 0.005;
      a_contact[i].surface.soft_erp = 0.9;
      a_contact[i].surface.soft_cfm = 0.001;
    
      dJointID joint_id = 
	dJointCreateContact(ode_handle.world, 
			    ode_handle.jointGroup,
			    &a_contact[i]);

      dJointAttach(joint_id ,
		   dGeomGetBody(a_contact[i].geom.g1), 
		   dGeomGetBody(a_contact[i].geom.g2));
    }
    */
  }


  for(++it;
      it != component_container.end();
      ++it) {
  
    if(true == (*it)->collision_callback(p_ode_handle, geom_id_0, geom_id_1))
      return true;
  }
  return false;
}



}
}


