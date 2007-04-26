#ifndef _CMDLINE_H
#define _CMDLINE_H

#include <selforg/configurable.h> 
#include <vector>

typedef vector<Configurable*> ConfigList; 

std::vector<string> splitString(const std::string& str, char seperator);

/// Shows the values of all parameters of the given configurable objects.
// @param file print to file or stdout if NULL
// @param lineprefix is used as prefix for each line if not NULL
void showParams(const ConfigList& configs, FILE* file = 0, const char* lineprefix = 0);
/// Asks the user for changing a parameter of the configurable objects.
// ALL input and output goes to stdin and stdout. 
// @param onTerminate() callback if user quits the program
void changeParams(ConfigList& configs, void (*onTerminate)()=0);

bool control_c_pressed();
void cmd_handler_init();
void cmd_end_input();

#endif
