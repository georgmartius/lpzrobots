#!BPY
# -*- coding: utf-8 -*-
"""
Name: 'treegenerator'
Blender: 237
Group: 'Wizards'
Tooltip: 'Tree generator by Josef Grunig, modified by Stefan Birgmeier'
"""

#/***************************************************************************
# *   Copyright (C) 2004 by Josef Grunig                                    *
# *   josef_grunig@yahoo.it                                                 *
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU General Public License as published by  *
# *   the Free Software Foundation; either version 2 of the License, or     *
# *   (at your option) any later version.                                   *
# *                                                                         *
# *   This program is distributed in the hope that it will be useful,       *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU General Public License for more details.                          *
# *                                                                         *
# *   You should have received a copy of the GNU General Public License     *
# *   along with this program; if not, write to the                         *
# *   Free Software Foundation, Inc.,                                       *
# *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
# ***************************************************************************/

#################################
#                               #
#         T R E E G E N         #
#                               #
#        by Josef Grunig        #
#       -----------------       #
#                               #
#    execute using:   Alt-P     #
#################################
#                               #
#         modified my           #
#       Stefan Birgmeier        #
#                               #
#################################


import Blender
from Blender import NMesh
from Blender.BGL import *
from Blender import Mathutils
from Blender.Mathutils import *
from Blender.Window import DrawProgressBar
from Blender.Draw import *
from Blender import Material, Texture, Image
import math
from math import *
import random

#DEFAULT DIR#
defaultDir = "/home/guettler/workspace/lpzRobots/ode_robots/simulations/template_XMLImportSim"

#VALORI INIZIALI DELLA GUI
N=10
BASE_FACTOR=1.5
TREE_LENGTH=30.0
RANDOM_GROW_FACTOR = 10
TREE_GROW_SPEED=2
LEAF_DEPTH = 1
LEAF_PER_NODE = 1
LEAF_FACTOR = 1.0
INITIAL_BRANCH_FREQUENCY = 3
NUM_BRANCH_PER_NODE = 1
MIN_BRANCH_ANGLE = 50
MAX_BRANCH_ANGLE = 70
INITIAL_MIN_BRANCH_HEIGHT = 10
CLOSURE_MIN_BRANCH_HEIGHT = 0.8
CLOSURE_BRANCH_FREQUENCY = 0.5
CLOSURE_GROW_FACTOR = 0.95
CLOSURE_BRANCH_FACTOR = 0.6
CLOSURE_BRANCH_LENGTH_FACTOR = 0.7
CLOSURE_GROW_SPEED_FACTOR = 0.8
CLOSURE_VERTICAL_LENGTH_FACTOR = 0.995
MAX_DEPTH=3

SELECT_LEAF_TEXTURE = 331
SELECT_STEM_TEXTURE = 332
EVT_NOTHING = 0
EVT_LEAFSIZE = 333

maxDepthButton = Create(MAX_DEPTH)
baseResolutionButton = Create(N)
baseFactorButton = Create(BASE_FACTOR)
treeLengthButton = Create(TREE_LENGTH)
treeGrowSpeedButton = Create(TREE_GROW_SPEED)
randomGrowFactorButton = Create(RANDOM_GROW_FACTOR)
leafDepthButton = Create(LEAF_DEPTH)
leafPerNodeButton = Create(LEAF_PER_NODE)
leafFactorButton = Create(LEAF_FACTOR)
initialBranchFrequencyButton = Create(INITIAL_BRANCH_FREQUENCY)
minBranchAngleButton = Create(MIN_BRANCH_ANGLE)
maxBranchAngleButton = Create(MAX_BRANCH_ANGLE)
initialMinBranchHeightButton = Create(INITIAL_MIN_BRANCH_HEIGHT)
closureMinBranchHeightButton = Create(CLOSURE_MIN_BRANCH_HEIGHT)
closureBranchFrequencyButton = Create(CLOSURE_BRANCH_FREQUENCY)
closureGrowFactorButton = Create(CLOSURE_GROW_FACTOR)
closureBranchFactorButton = Create(CLOSURE_BRANCH_FACTOR)
closureBranchLengthFactorButton = Create(CLOSURE_BRANCH_LENGTH_FACTOR)
closureTreeGrowSpeedButton = Create(CLOSURE_GROW_SPEED_FACTOR)
closureVerticalLengthFactorButton = Create(CLOSURE_VERTICAL_LENGTH_FACTOR)
numBranchButton = Create(NUM_BRANCH_PER_NODE)
treeExampleMenu = Create(1)
defaultDirButton = Create(defaultDir)
defaultDirButtonStem = Create(defaultDir)
leafTextureMenu = Create(1)
stemTextureMenu = Create(1)
leafTexture = "Leaf.png"
stemTexture = "stamm.png"


sel_leaf_file = Create("Select leaf texture")
sel_stem_file = Create("Select stem texture")
leafSizeX = Create(2)
leafSizeY = Create(2)

def selectfile(file):
	defaultDirButton.val = file
	
def selectfilestem(file_stem):
	defaultDirButtonStem.val = file_stem
	
def select_l_texture():
	Blender.Window.FileSelector(selectfile, "Select leaf texture", defaultDir)

def select_s_texture():
	Blender.Window.FileSelector(selectfilestem, "Select stem texture", defaultDir)



def makeLeaf(rotation, translation):
  vector = Vector([- leafSizeX.val / 2 * leafFactorButton.val, 0 , 0 * leafFactorButton.val, 1])
  vector = MatMultVec(rotation,vector)
  vector = VecMultMat(vector, translation)
  v = NMesh.Vert(vector.x, vector.y, vector.z)
  foglie.verts.append(v)

  vector = Vector([leafSizeX.val / 2 * leafFactorButton.val, 0 , 0 * leafFactorButton.val, 1])
  vector = MatMultVec(rotation,vector)
  vector = VecMultMat(vector, translation)
  v = NMesh.Vert(vector.x, vector.y, vector.z)
  foglie.verts.append(v)

  vector = Vector([leafSizeX.val / 2 * leafFactorButton.val, 0, leafSizeY.val * leafFactorButton.val, 1])
  vector = MatMultVec(rotation,vector)
  vector = VecMultMat(vector, translation)
  v = NMesh.Vert(vector.x, vector.y, vector.z)
  foglie.verts.append(v)

  vector = Vector([- leafSizeX.val / 2 * leafFactorButton.val, 0, leafSizeY.val * leafFactorButton.val, 1])
  vector = MatMultVec(rotation,vector)
  vector = VecMultMat(vector, translation)
  v = NMesh.Vert(vector.x, vector.y, vector.z)
  foglie.verts.append(v)
  

def creaTronco(length,depth,min_length, branch_frequency, direction,rotTotal,traslTotal, closure_grow_factor, nTasselli, treeGrowSpeed):
  n = baseResolutionButton.val
  cont_nodo=0
  for actual_length in range(0,length):
    if (depth==0):
      DrawProgressBar(actual_length / treeLengthButton.val, "Creating Tree...")
    cont_nodo=cont_nodo+1
    #nTasselli conta il numero di pezzi da disegnare in seguito
    nTasselli = nTasselli +1
    #Aggiorno fattore di crescita
    closure_grow_factor = closure_grow_factor * closureGrowFactorButton.val
    #Ciclo di creazione tronco
    #print actual_length

    #Randomizzazione della direzione del tassello
    #Trovo due angoli random   
    angle1 = random.uniform(-randomGrowFactorButton.val, randomGrowFactorButton.val)
    angle2 = random.uniform(-randomGrowFactorButton.val, randomGrowFactorButton.val)
    rotmat1 = RotationMatrix(angle1, 4, 'x')
    rotmat2 = RotationMatrix(angle2, 4, 'y')
    directionRot = rotmat1 * rotmat2
    rotMat = rotmat1 * rotmat2 * rotTotal
    direction = MatMultVec(directionRot, direction)
    #print direction
    trasl = TranslationMatrix(direction)
    traslTotal = traslTotal * trasl
    
    #Creo nuovo tassello
    for i in range(0, n):
    
      x = (tronco.verts[i].co[0]) * closure_grow_factor
      y = (tronco.verts[i].co[1]) * closure_grow_factor
      z = (tronco.verts[i].co[2])
      vector = Vector ([x,y,z,1])
      vector = MatMultVec(rotMat,vector)
      vector = VecMultMat(vector, traslTotal)
      v = NMesh.Vert(vector.x, vector.y, vector.z)
      tronco.verts.append(v)
     
    #Possibile diramazione
    if ((actual_length > min_length) and (depth < maxDepthButton.val) and (cont_nodo>= branch_frequency)):
      
      for nBranch in range(0,numBranchButton.val):
        #Azzero il contatore della frequenza dei Tasselli
        cont_nodo=0
        #Calcolo nuova direzione
        #Randomizzazione della direzione del tassello
        #Trovo due angoli random   
        anglex = random.uniform(minBranchAngleButton.val, maxBranchAngleButton.val)
        anglez = random.uniform(0,360)
        rotmatx = RotationMatrix(anglex, 4, 'x')
        rotmatz = RotationMatrix(anglez, 4, 'z')
        newRotTotal = rotMat * rotmatz * rotmatx
        newDirection = MatMultVec(newRotTotal, Vector([0,0,treeGrowSpeed,1]))
      

        nTasselli = creaTronco(pow(closureVerticalLengthFactorButton.val, actual_length) * length * closureBranchLengthFactorButton.val,depth+1, min_length * closureMinBranchHeightButton.val, branch_frequency * closureBranchFrequencyButton.val, newDirection,newRotTotal, traslTotal, closure_grow_factor*closureBranchFactorButton.val, nTasselli, treeGrowSpeed * closureTreeGrowSpeedButton.val)
        #print "Ritorno"
        #actual_length=6
    
        #Creo tassello di giunzione
        nTasselli = nTasselli +1
        for i in range(0, n):
          v = NMesh.Vert(0, 0, 0)
          tronco.verts.append(v)
        
        nTasselli = nTasselli +1
        for i in range(0, n):
    
          x = (tronco.verts[i].co[0]) * closure_grow_factor
          y = (tronco.verts[i].co[1]) * closure_grow_factor
          z = (tronco.verts[i].co[2])
          vector = Vector ([x,y,z,1])
          vector = MatMultVec(rotTotal,vector)
          vector = VecMultMat(vector, traslTotal)
          v = NMesh.Vert(vector.x, vector.y, vector.z)
          tronco.verts.append(v)


    #Possibile Foglia
    if (depth >= leafDepthButton.val):
      for i in range(0, leafPerNodeButton.val):
        #print "Foglia"

        #Calcolo rotazione foglia
        #Trovo tre angoli random   
        anglex = random.uniform(-40, 40)
        angley = random.uniform(-40, 40)
        anglez = random.uniform(0, 360)
        rotmatx = RotationMatrix(anglex, 4, 'x')
        rotmaty = RotationMatrix(angley, 4, 'y')
        rotmatz = RotationMatrix(anglez, 4, 'z')
        newLeafRotTotal = rotMat * rotmatz * rotmaty * rotmatx
        makeLeaf(newLeafRotTotal, traslTotal)

  #
    
  #
  #print nTasselli
  return(nTasselli)
  
    
#

def makeNewTree():
	randomSeed = random.getstate()
	makeTree()
#
#FINE makeNewTree

def makeSameTree():
	random.setstate(randomSeed)
	makeTree()
#
#FINE makeSameTree()


def makeTree():
  #INIZIO PROGRAMMA
  print "-------------------"
  global leafTexture, stemTexture
  global tronco, foglie
  n = baseResolutionButton.val
  #Creo Mesh del tronco
  tronco = NMesh.GetRaw("stemMesh")
  if (tronco==None): tronco = NMesh.GetRaw()
  tronco.verts = []
  tronco.faces = []

  #Creo Mesh delle foglie
  foglie = NMesh.GetRaw("leafMesh")
  if (foglie==None): foglie = NMesh.GetRaw()
  foglie.verts = []
  foglie.faces = []

  #Creo la base del tronco
  for i in range(0,n):
    x = baseFactorButton.val * cos(2*pi*i/(n-1))
    y = baseFactorButton.val * sin(2*pi*i/(n-1))
    z = 0
    v = NMesh.Vert(x, y, z)
    tronco.verts.append(v)
  #


  direction = Vector([0,0,treeGrowSpeedButton.val,1])
  traslTotal=TranslationMatrix(direction)
  rotTotal=RotationMatrix(0,4,'z')

  nTasselli = creaTronco(treeLengthButton.val, 0, initialMinBranchHeightButton.val, initialBranchFrequencyButton.val, direction, rotTotal,traslTotal, closureGrowFactorButton.val, 0, treeGrowSpeedButton.val * closureTreeGrowSpeedButton.val)


  progressMAX = nTasselli * 1.0
  #Drawing Faces TRONCO
  for j in range(0, nTasselli):
    #Al posto di 1000 mettere nTasselli/numeroRedraw (???Non so come trasformare in intero)
    if ((j % 1000)==0):
      DrawProgressBar(j / progressMAX, "Drawing Trunc...")
    for i in range(0, n-1):
      #print "faccia:"
      #print i
      #Evito di disegnare i punti collassati in [0,0,0] inseriti per accorgermi della giunzione
      if ((tronco.verts[(j+1) * n + i +1].co[0]==0) and (tronco.verts[(j+1) * n + i +1].co[1]==0) and (tronco.verts[(j+1) * n + i +1].co[2]==0)):
        continue
      if ((tronco.verts[(j) * n + i +1].co[0]==0) and (tronco.verts[(j) * n + i +1].co[1]==0) and (tronco.verts[(j) * n + i +1].co[2]==0)):
        continue
      f = NMesh.Face()
      f.v.append(tronco.verts[j * n + i])
      f.v.append(tronco.verts[j * n+ i + 1])
      f.v.append(tronco.verts[(j+1) * n + i +1])
      f.v.append(tronco.verts[(j+1) * n + i])
      tronco.faces.append(f)
    #

  #Drawing Faces LEAF
  cont=0
  numVert=0
  numFace=0
  f = NMesh.Face()
  nFoglie = len(foglie.verts)*1.0
  actual_nFoglie=0.0
  for vert in foglie.verts:
    #modificare anche qui come sopra
    if ((actual_nFoglie % 10000)==0):
      DrawProgressBar(actual_nFoglie / nFoglie, "Drawing Leaf...")
    actual_nFoglie +=1
    numVert += 1
    #print "Aggiungo vertice"
    f.v.append(vert)
    cont += 1
    if (cont==4):
      f.uv.append((0,0))
      f.uv.append((1,0))
      f.uv.append((1,1))
      f.uv.append((0,1))
      foglie.faces.append(f)
      f = NMesh.Face()
      #print "Stampo Faccia"
      #print numFace
      numFace += 1
      cont=0
  #

  #Materiali, Texture, e update
  if ((not tronco.materials) and (not foglie.materials)):
    #Primo avvio: inserire materiali etcc..

    #Materiale e texture Foglia
    leafMat = Material.New("leafMaterial")
    leafMat.setAlpha(0)
    leafMat.setRGBCol([0,0.3,0])
    leafMat.setSpecCol([0,0,0])
    leafMat.mode |= Material.Modes.ZTRANSP
    
    leafTex = Texture.New("leafTexture")
    leafTex.setType("Image")
    image = Image.Load(defaultDirButton.val)
    leafTex.image = image
    leafTex.setImageFlags("MipMap", "UseAlpha")
       
    leafMat.setTexture(0, leafTex)
    
    leafMTexList = leafMat.getTextures()
    leafMTex = leafMTexList[0]
    leafMTex.texco = Texture.TexCo.UV
    leafMTex.mapto |= Texture.MapTo.ALPHA
    
    #Associo materiale
    foglie.materials.append(leafMat)
    
    #Materiale Tronco
    stemMat = Material.New("stemMaterial")
    stemMat.setSpecCol([0,0,0])
        
    stemTex = Texture.New("stemTexture")
    stemTex.setType("Image")

    
    image = Image.Load(defaultDirButtonStem.val)
    stemTex.image = image
    stemTex.setImageFlags("MipMap")
       
    stemMat.setTexture(0, stemTex)
    
    stemMTexList = stemMat.getTextures()
    stemMTex = stemMTexList[0]
    stemMTex.texco = Texture.TexCo.UV
        
    
    #Associo materiale
    tronco.materials.append(stemMat)
    
    NMesh.PutRaw(tronco, "stemMesh", 1)
    NMesh.PutRaw(foglie, "leafMesh", 1)
  

  else:
    #Neccessito solo di cambiare le immagini texture
    leafMat = foglie.materials[0]
    leafMTexList = leafMat.getTextures()
    leafMTex = leafMTexList[0]
    image = Image.Load(defaultDirButton.val)
    leafTex = leafMTex.tex
    leafTex.image = image

    stemMat = tronco.materials[0]
    stemMTexList = stemMat.getTextures()
    stemMTex = stemMTexList[0]
    image = Image.Load(defaultDirButtonStem.val)
    stemTex = stemMTex.tex
    stemTex.image = image

    tronco.update()
    foglie.update()

  DrawProgressBar(1.0, "Finished")

  #Blender.Redraw()

#FINE makeTree()




def DrawGUI():
  global maxDepthButton, baseResolutionButton, baseFactorButton, treeLengthButton, treeGrowSpeedButton
  global closureBranchLengthFactorButton, randomGrowFactorButton, leafDepthButton, leafPerNodeButton
  global leafFactorButton, initialBranchFrequencyButton, closureBranchFrequencyButton, initialMinBranchHeightButton
  global closureMinBranchHeightButton, minBranchAngleButton, maxBranchAngleButton, closureGrowFactorButton
  global closureBranchFactorButton, numBranchButton, treeExampleMenu, closureTreeGrowSpeedButton
  global closureVerticalLengthFactorButton, defaultDirButton, defaultDirButtonStem, leafTextureMenu, stemTextureMenu
  global sel_leaf_file, sel_stem_file, leafSizeX, leafSizeY
  global randomSeed

  defaultDirButton = String("Leaf Texture: ", EVT_NOTHING, 5, 500, 300, 20, defaultDirButton.val,200,"Path to the leaf texture")
  sel_leaf_file = PushButton("Select leaf texture", SELECT_LEAF_TEXTURE, 305, 500, 140, 20, "Select a leaf texture")

  defaultDirButtonStem = String("Stem Texture: ", EVT_NOTHING, 5, 475, 300, 20, defaultDirButtonStem.val, 200, "Path to the stem texture")
  sel_stem_file = PushButton("Select stem texture", SELECT_STEM_TEXTURE, 305, 475, 140, 20, "Select a stem texture")

  glRasterPos2d(5, 455)
  Text("Stem Properties:")

  baseResolutionButton = Slider("Resolution: ", 0, 5, 430, 220, 20, baseResolutionButton.val, 4, 100, 0, "Number of points")
  maxDepthButton = Slider("TreeDepth: ", 0, 230, 430, 220, 20, maxDepthButton.val, 0, 9, 0, "Depth of the tree")
  
  baseFactorButton = Slider("BaseScale: ", 0, 5, 410, 220, 20, baseFactorButton.val, 0.1, 10.0, 0, "Factor which multiplies the dimension of the base of the tree")
  randomGrowFactorButton = Slider("RandDirection: ", 0, 230, 410, 220, 20, randomGrowFactorButton.val, 0, 90, 0, "In degrees, indicates how much can the direction change")
 
  treeLengthButton = Slider("TreeLength: ", 0, 5, 390, 220, 20, treeLengthButton.val, 1, 100, 0, "Length of the tree")
  closureBranchLengthFactorButton = Slider("LengthScale: ", 0, 230, 390, 220, 20, closureBranchLengthFactorButton.val, 0.0, 1.0, 0, "How much, each branch, the length should decrease")

  closureGrowFactorButton = Slider("StemScale: ", 0, 5, 370, 220, 20, closureGrowFactorButton.val, 0.001, 1.000, 0, "Every primitive segment the stem decreases about this factor")
  closureBranchFactorButton = Slider("BranchScale: ", 0, 230, 370, 220, 20, closureBranchFactorButton.val, 0.001, 1.000, 0, "Every branch the stem decreases about this factor")
  
  treeGrowSpeedButton = Slider("GrowSpeed: ", 0, 5, 350, 220, 20, treeGrowSpeedButton.val, 0.1, 10.0, 0, "Indicates how long are the primitive segments")
  closureTreeGrowSpeedButton = Slider("GrowSpDecr: ", 0, 230, 350, 220, 20, closureTreeGrowSpeedButton.val, 0.0, 1.0, 0, "Factor that decreases the length of the primitives segments")

  closureVerticalLengthFactorButton = Slider("VertLenFact: ", 0, 5, 330, 220, 20, closureVerticalLengthFactorButton.val, 0.95, 1.0, 0, "Factor that decreases the length of branches in a proportional way to the heigth of the tree")
  
##  stemTextureMenu = Menu("Stem Texture %t | Gray Stem %x1 | Brown Stem %x2",5,230,330,220,20,stemTextureMenu.val,"Select a stem texture")

  glRasterPos2d(5, 300)
  Text("Leaf Properties:")

  leafDepthButton = Slider("LeafDepth: ", 0, 5, 270, 220, 20, leafDepthButton.val, 0, 9, 0, "Depth where the leafes start growing")
  leafPerNodeButton = Slider("LeafPerNode: ", 0, 230, 270, 220, 20, leafPerNodeButton.val, 0, 10, 0, "How many leafes should grow each primitive segment")

  leafFactorButton = Slider("LeafScale: ", 0, 5, 250, 220, 20, leafFactorButton.val, 0.01, 2.00, 0, "Multiplies the BaseLeaf dimension")
  
  leafSizeX = Slider("Leaf size X: ", EVT_LEAFSIZE, 5, 225, 220, 20, leafSizeX.val, 0.5, 10, 0, "Leaf X dimension")
  
  leafSizeY = Slider("Leaf size Y: ", EVT_LEAFSIZE, 5, 200, 220, 20, leafSizeY.val, 0.5, 10, 0, "Leaf Y dimension")
  
##  leafTextureMenu = Menu("Leaf Texture %t | Acacia %x1 | Cherry %x2 | Fig %x3 | Hazel %x4 | Oak %x5 | Walnut %x6",4,230,250,220,20,leafTextureMenu.val,"Select a leaf texture")


  glRasterPos2d(5, 175)
  Text("Branch Properties:")

  initialBranchFrequencyButton = Slider("Freq: ", 0, 5, 150, 220, 20, initialBranchFrequencyButton.val, 1, 100, 0, "Branch every BranchFreq primitive segments")
  closureBranchFrequencyButton = Slider("FreqDecr: ", 0, 230, 150, 220, 20, closureBranchFrequencyButton.val, 0.0, 1.0, 0, "Every Branch BranchFreq Drecreses about BranchFreqDrecrese Factor")

  initialMinBranchHeightButton = Slider("MinHeight: ", 0, 5, 125, 220, 20, initialMinBranchHeightButton.val, 0, 100, 0, "The minimal height to be reached before first branch")
  closureMinBranchHeightButton = Slider("MinHDecr: ", 0, 230, 125, 220, 20, closureMinBranchHeightButton.val, 0.0, 1.0, 0, "Every branch the minimal height changes about this factor")

  minBranchAngleButton = Slider("MinAngle: ", 0, 5, 100, 220, 20, minBranchAngleButton.val, 0, 180, 0, "Minimal Vertical Branch Angle")
  maxBranchAngleButton = Slider("MaxAngle: ", 0, 230, 100, 220, 20, maxBranchAngleButton.val, 0, 180, 0, "Maximal Vertical Branch Angle")
  
  numBranchButton = Slider("NumBranch: ", 0, 5, 75, 220, 20, numBranchButton.val, 0, 10, 0, "Maximal Vertical Branch Angle")
    
##  glRasterPos2d(5, 100)
##  Text("Tree Examples:")

##  treeExampleMenu = Menu("Tree Examples %t | Vine Tree %x1 | Hazel Tree %x2 | Acacia Tree %x3 | Oak Tree %x4",3,5,70,220,20,treeExampleMenu.val,"Some example of common trees")


  Button("EXIT", 1, 5, 10, 40, 25)
  Button("GENERATE (same seed)", 2, 60, 10, 160, 25)
  Button("GENERATE (new seed)", 3, 230, 10, 160, 25)

  randomSeed = random.getstate()

def event(evt, val):
  if (0):
    print 

def buttonEvent(evt):
	if (evt==0): Draw
	
	elif (evt==1): Exit()
	
	elif (evt==2): makeSameTree()
	
	elif (evt==3): makeNewTree()

	elif (evt == SELECT_LEAF_TEXTURE):
		select_l_texture()
	elif (evt == SELECT_STEM_TEXTURE):
		select_s_texture()



Register(DrawGUI, event, buttonEvent)


#Creo Albero
#makeTree()