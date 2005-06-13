
// TODO
// textures directory needed

#include <stdio.h>
#include <math.h>
#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include "simulation.h"

#include "odeconfig.h"

//Steuerungsvariablen
double SimulationTime = 0;
int sim_step = 0;

#include "invertnchannelcontroller.h"
AbstractController *controller;

#include <noisegenerator.h>
NoiseGenerator<2> noise_gen;

OdeConfig config;
const int configs_len=2; // number of configurable objects
Configurable* configs[configs_len];

static dWorldID welt;
static dSpaceID raum;
static dJointGroupID contactgroup;

dGeomID untergrund; //Untergundfläche


#include "playground.h"
Playground playground(&welt, &raum);

#include "vehicle_2wheels.h"
Vehicle vehicle(&welt, &raum);



// Funktion die die Steuerung des Roboters uebernimmt
bool StepRobot()
{
  double x[2];
  double y[2];

  vehicle.getSensors(x,2);
//   for (int i=0; i<2; i++){
//     x[i]*=0.05;
//   }
  // uniformly distributed noise -> using min=-noise, max=noise
  noise_gen.addColoredUniformlyDistributedNoise(x, -config.noise, config.noise);   
   
  controller->step(x,2,y,2)   ;

//   for (int i=0; i<2; i++){
//     y[i]*=20.0;
//   }
  vehicle.setMotors(y,2);

  return( 1 );
}


//Diese Funktion wird immer aufgerufen, wenn es im definierten Space zu einer Kollission kam
//Hier wird die Kollission näher untersucht
void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  int i,n;

  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++)
      {
	contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	  dContactSoftERP | dContactSoftCFM | dContactApprox1;
//         if ((o1=vehicle.box[0]) || (o2=vehicle.box[0]) ){
// 	  contact[i].surface.mu = 0.2;//0.8; //normale Reibung von Reifen auf Asphalt
// 	}else{
// 	  contact[i].surface.mu = 2.0;//0.8; //normale Reibung von Reifen auf Asphalt
// 	}
	contact[i].surface.mu = 0.8; //normale Reibung von Reifen auf Asphalt
	contact[i].surface.slip1 = 0.005;
	contact[i].surface.slip2 = 0.005;
	contact[i].surface.soft_erp = 1;
	contact[i].surface.soft_cfm = 0.00001;
	dJointID c = dJointCreateContact (welt,contactgroup,&contact[i]);
	dJointAttach ( c , dGeomGetBody(contact[i].geom.g1) , dGeomGetBody(contact[i].geom.g2)) ;
	
      }
  }
}



//Startfunktion die am Anfang der Simulationsschleife, einmal ausgefuehrt wird
void start() 
{
  //Anfangskameraposition und Punkt auf den die Kamera blickt
  float KameraXYZ[3]= {2.1640f,-1.3079f,1.7600f};
  float KameraViewXYZ[3] = {125.5000f,-17.0000f,0.0000f};;
  dsSetViewpoint ( KameraXYZ , KameraViewXYZ );
  dsSetSphereQuality (2); //Qualitaet in der Sphaeren gezeichnet werden

  // initialization
  config.noise=0.1;

  cmd_handler_init();

  dsPrint ( "\n\nWillkommen beim virtuellen Roboterarmsimulator:\n" );
  dsPrint ( "---------------------------------------------------------------------------------------\n" );
  dsPrint ( "Druecken sie h fuer die Hilfe.\nCtrl-C fuer Aenderung von Parametern\n\n" );

  configs[0]=&config;
  configs[1]=controller;
  showParams(configs, configs_len);
}

void end(){
}


//Schleife der Simulation
void simLoop ( int pause )
{
  // Parametereingabe  
  if (control_c_pressed()){
    cmd_begin_input();
    changeParams(configs, configs_len);
    cmd_end_input();
  }

  //die Simulation wird nur weitergeführt wenn keine Pause aktiviert wurde
  if (!pause) {
    //**************************Steuerungsabschnitt ************************
    SimulationTime += config.simStepSize;
    
    sim_step = sim_step + 1;
    if ( (sim_step % config.controlInterval ) == 0 )
      StepRobot ();
  
    /**********************Simulationsschritt an sich**********************/
    dSpaceCollide ( raum , 0 , &nearCallback );
    dWorldStep ( welt , config.simStepSize ); //ODE-Engine geht einen Schritt weiter
    dJointGroupEmpty (contactgroup);
    
    if(sim_step % config.drawInterval == 0){
      /**************************Zeichenabschnitt***********************/
      playground.draw(); // box zeichnen
      vehicle.draw();
    }
  }

}

int main (int argc, char **argv)
{

  controller=getController(2,10);
  
  /**************************Grafikabschnitt**********************/
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = 0;
  fn.stop = &end;
  fn.path_to_textures = 0;

  /***************************ODE-Abschnitt***********************/

  //****************Weltdefinitionsabschnitt**************
  //Anlegen der Welt an sich
  welt = dWorldCreate ();

  //Anlegen eines Raumes der Welt in der Sichtbare Koerper eine raeumliche Ausdehnung annehmen koennen
  //ist fuer die Kollissionserkennung wichitg
  raum = dHashSpaceCreate (0);

  contactgroup = dJointGroupCreate ( 1000000 );

  //Weltgravitataet
  dWorldSetGravity ( welt , 0 , 0 , -9.81 );
  dWorldSetERP ( welt , 1 );
  untergrund = dCreatePlane ( raum , 0 , 0 , 1 , 0 );
		
  playground.setGeometry(7.0, 0.2, 1.5);
  playground.setPosition(0,0,0); // playground positionieren und generieren

  dVector3 v;
  mkVector(v,-1,0,0);
  vehicle.setPosition(v);
  vehicle.create();

  //********************Simmulationsstart*****************
  dsSimulationLoop ( argc , argv , 500 , 500 , &fn );

  //******Speicherfreigabe, Welt- und Raumzerstoerung*****
  dJointGroupDestroy ( contactgroup );
  dWorldDestroy ( welt );
  dSpaceDestroy ( raum );
  dCloseODE ();
  return 0;
}
