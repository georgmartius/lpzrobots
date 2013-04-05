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
 *  Revision 1.4  2010-03-11 15:18:06  guettler
 *  -BoundingShape can now be set from outside (see XMLBoundingShape)
 *  -Mesh can be created without Body and Geom.
 *  -various bugfixes
 *
 *  Revision 1.3  2010/03/10 13:54:59  guettler
 *  further developments for xmlimport
 *
 *  Revision 1.2  2010/03/08 07:18:56  guettler
 *  StandardCamera renamed to StandardMode
 *
 *  Revision 1.1  2010/03/07 22:50:38  guettler
 *  first development state for feature XMLImport
 *                                                                                   *
 *                                                                         *
 **************************************************************************/
#ifndef __XMLDEFINITIONS_H_
#define __XMLDEFINITIONS_H_

namespace XMLDefinitions {

  const static double compareEPS               = 1e-12;

  // global nodes
  const static char* sceneNode                = "Scene";
  const static char* globalVariablesNode      = "GlobalVariables";
  const static char* passiveObjectsNode       = "PassiveObjects";
  const static char* agentsNode               = "Agents";
  const static char* cameraNode               = "Camera";
  const static char* lightsNode               = "Lights";


  // global variables
  const static char* noiseAtt                 = "Noise";
  const static char* gravityAtt               = "Gravity";
  const static char* realTimeFactorAtt        = "RealtimeFactor";
  const static char* controlintervalAtt       = "ControlInterval";
  const static char* simStepSizeAtt           = "SimStepSize";
  const static char* randomSeedAtt            = "RandomSeed";
  const static char* fpsAtt                   = "Fps";
  const static char* motionPersistenceAtt     = "MotionPersistence";
  const static char* cameraSpeedAtt           = "CameraSpeed";
  const static char* drawBoundingsAtt         = "DrawBoundings";
  const static char* shadowTypeAtt            = "ShadowType";


  // Primitives
  const static char* boxNode                  = "Box";
  const static char* capsuleNode              = "Capsule";
  const static char* cylinderNode             = "Cylinder";
  const static char* dummyPrimitiveNode       = "DummyPrimitive";
  const static char* meshNode                 = "Mesh";
  const static char* planeNode                = "Plane";
  const static char* rayNode                  = "Ray";
  const static char* sphereNode               = "Sphere";
  const static char* transformNode            = "Transform";
  const static char* playgroundNode           = "Playground";


  // physical representation
  const static char* massAtt                  = "Mass";
  const static char* materialNode             = "Material";
  const static char* radiusAtt                = "Radius";
  const static char* lengthAtt                = "Length";
  const static char* widthAtt                 = "Width";
  const static char* heightAtt                = "Height";
  const static char* permeableAtt             = "Permeable";
  const static char* visibleAtt               = "Visible";

  // graphical representation
  const static char* graphicalRepresentationNode = "GraphicalRepresentation";
  const static char* fileAtt                  = "Filename";
  const static char* texture                  = "Texture";
  const static char* repeatOnRAtt             = "wrapTextureOnR";
  const static char* repeatOnSAtt             = "wrapTextureOnS";
  const static char* surfaceAtt               = "surface";
  const static char* elasticityAtt            = "Elasticity";
  const static char* roughnessAtt             = "Roughness";
  const static char* slipAtt                  = "Slip";
  const static char* hardnessAtt              = "Hardness";

  // position, rotation, scale
  const static char* positionNode             = "Position";
  const static char* viewPositionNode         = "ViewPosition";
  const static char* rotationNode             = "Rotation";
  const static char* geometryNode             = "Geometry";
  const static char* scaleAtt                 = "Scaling";
  const static char* xAtt                     = "X";
  const static char* yAtt                     = "Y";
  const static char* zAtt                     = "Z";
  const static char* alphaAtt                 = "Alpha";
  const static char* betaAtt                  = "Beta";
  const static char* gammaAtt                 = "Gamma";

  // color (and their values)
  const static char* colorNode                = "Color";
  const static char* redAtt                   = "Red";
  const static char* greenAtt                 = "Green";
  const static char* blueAtt                  = "Blue";
  const static char* alphacolorAtt            = "Alpha";

  // for Joints and Transforms
  const static char* parentPrimitive                            = "PrimitiveId1";
  const static char* childPrimitive                               = "PrimitiveId2";

  // Primitive ray
  const static char* rangeAtt                        = "Range";
  const static char* thicknessAtt                                      = "Thickness";

  // Mesh related nodes
  const static char* boundingShapeNode        ="BoundingShape";

  // Agent and Robot
  const static char* agentNode                ="Agent";
  const static char* robotNode                ="Robot";

  // Controller

  // Wiring

  // Camera related nodes
  const static char* cameraStandardModeNode   = "StandardMode";

  // Light
};

#endif /* __XMLDEFINITIONS_H_ */
