/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    guettler@informatik.uni-leipzig.de                                   *
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
 ***************************************************************************
 *                                                                         *
 *  DESCRIPTION                                                            *
 *  
 *                                                                         *
 *                                                                         *
 *  $Log$
 *  Revision 1.1  2010-03-07 22:50:38  guettler
 *  first development state for feature XMLImport
 *										   *
 *                                                                         *
 **************************************************************************/
#ifndef __XMLDEFINITIONS_H_
#define __XMLDEFINITIONS_H_

namespace XMLDefinitions {

  // global nodes
  const static char* sceneNode                = "scene";
  const static char* objectsNode              = "objects";


  // obstacles
  const static char* PassiveMeshNode          = "PassiveMesh";
  const static char* PassiveBoxNode           = "PassiveBox";
  const static char* PassiveSphereNode        = "PassiveSphere";
  const static char* PassiveCapsuleNode       = "PassiveCapsule";
//  const static char* PassiveCylinderNode      = "PassiveCylinder";
  const static char* TerrainGroundNode        = "TerrainGround";
  const static char* MeshGroundNode           = "PassiveBox";


  // Primitives
  const static char* BoxNode                  = "Box";
  const static char* CapsuleNode              = "Capsule";
  const static char* CylinderNode             = "Cylinder";
  const static char* DummyPrimitiveNode       = "DummyPrimitive";
  const static char* MeshNode                 = "Mesh";
  const static char* PlaneNode                = "Plane";
  const static char* RayNode                  = "Ray";
  const static char* SphereNode               = "Sphere";
  const static char* TransformNode            = "Transform";
  const static char* MaterialNode              = "Material";

  // physical representation
  const static char* massAtt                  = "Mass";

  // position, rotation, scale
  const static char* positionNode             = "Position";
  const static char* viewPositionNode         = "ViewPosition";
  const static char* rotationNode             = "Rotation";
  const static char* geometryNode             = "Geometry";
  const static char* scaleAtt                 = "scaling";
  const static char* xAtt                     = "X";
  const static char* yAtt                     = "Y";
  const static char* zAtt                     = "Z";
  const static char* alphaAtt                 = "Alpha";
  const static char* betaAtt                  = "Beta";
  const static char* gammaAtt                 = "Gamma";

  // color (and their values)
  const static char* colorNode                = "Color";
  const static char* redAtt                   = "red";
  const static char* greenAtt                 = "green";
  const static char* blueAtt                  = "blue";
  const static char* alphacolorAtt            = "alpha";

  // Primitive related attributes
  const static char* radiusAtt                = "Radius";
  const static char* lengthAtt                = "Length";
  const static char* widthAtt                 = "Width";
  const static char* heightAtt                = "Height";
  const static char* permeableAtt             = "permeable";
  const static char* visibleAtt               = "visible";



  // for Joints and Transforms
  const static char* parentPrimitive		    	= "PrimitiveId1";
  const static char* childPrimitive		       	= "PrimitiveId2";

  // Primitive ray
  const static char* rangeAtt                	= "range";
  const static char* thicknessAtt			      	= "thickness";


  // GraphicalRepresentation
  const static char* GraphicalRepresentationNode = "GraphicalRepresentation";
  const static char* fileAtt                  = "Filename";
  const static char* texture              	  = "Texture";
  const static char* repeatOnRAtt             = "wrapTextureOnR";
  const static char* repeatOnSAtt             = "wrapTextureOnS";
  const static char* surfaceAtt               = "surface";
  const static char* elasticityAtt  				  = "Elasticity";
  const static char* roughnessAtt	  			    = "Roughness";
  const static char* slipAtt			       		  = "Slip";
  const static char* hardnessAtt			  	    = "Hardness";

  // Mesh related nodes
  const static char* BoundingShapeNode        ="BoundingShape";

  const static char* GlobalVariablesNode      ="GlobalVariables";
  const static char* noiseAtt                 = "Noise";
  const static char* gravityAtt               = "Gravity";
  const static char* realTimeFactorAtt        = "RealtimeFactor";
  const static char* controlintervalAtt       = "ControlInterval";
  const static char* simStepSizeAtt           = "SimStepSize";
  const static char* randomSeedAtt            = "RandomSeed";
  const static char* fpsAtt                   = "Fps";
  const static char* motionPersistenceAtt     = "MotionPersistence";
  const static char* cameraSpeedAtt           = "CameraSpeed";
  const static char* DrawBoundingsAtt         = "DrawBoundings";
  const static char* shadowTypeAtt            = "ShadowType";

  // Camera related nodes
  const static char* cameraNode               = "Camera";
  const static char* standardCameraModeNode   = "StandardCamera";
};

#endif /* __XMLDEFINITIONS_H_ */
