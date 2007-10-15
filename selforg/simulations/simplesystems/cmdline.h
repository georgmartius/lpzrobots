#ifndef _CMDLINE_H
#define _CMDLINE_H

#include <selforg/configurable.h> 
#include <vector>

typedef std::vector<Configurable*> ConfigList; 

std::vector<std::string> splitString(const std::string& str, char seperator);

/// Shows the values of all parameters of the given configurable objects.
// @param file print to file or stdout if NULL
// @param lineprefix is used as prefix for each line if not NULL
void showParams(const ConfigList& configs, FILE* file = 0, const char* lineprefix = 0);

bool control_c_pressed();
void cmd_handler_init();
void cmd_end_input();

#endif
