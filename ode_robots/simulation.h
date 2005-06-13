#ifndef __SIMULATION_H
#define __SIMULATION_H

//dadurch wird mit den Double-Genauigkeitszeichenmethoden gearbeitet
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCappedCylinder dsDrawCappedCylinderD
#endif

#define PI M_PI // (3.14159265358979323846)

#include "configurable.h"

void showParams(Configurable** configs, int len);
void changeParams(Configurable** configs, int len);

void cmd_handler_init();
bool control_c_pressed();
void cmd_begin_input();
void cmd_end_input();






#endif
