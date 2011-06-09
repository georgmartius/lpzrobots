/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
 *    frankguettler@gmx.de                                                 *
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
 *    taken vom fileman.c from GNU readline lib                            *
 *    implements a cmd line interface using readline lib                   *
 *                                                                         *
 *   $Log$
 *   Revision 1.16  2011-06-09 13:26:26  martius
 *   added RandomObstacle and keys: o and O to add and remove them on the fly
 *
 *   Revision 1.15  2011/06/03 13:42:48  martius
 *   oderobot has objects and joints, store and restore works automatically
 *   removed showConfigs and changed deprecated odeagent calls
 *
 *   Revision 1.14  2011/05/30 21:57:16  martius
 *   store and restore from console improved
 *   console width automatically adapted
 *
 *   Revision 1.13  2011/05/30 13:56:42  martius
 *   clean up: moved old code to oldstuff
 *   configable changed: notifyOnChanges is now used
 *    getParam,setParam, getParamList is not to be overloaded anymore
 *
 *   Revision 1.12  2011/03/22 16:44:10  guettler
 *   - adpaptions to enhanced configurable and inspectable interface
 *   - better formatted output in showParams()
 *
 *   Revision 1.11  2011/03/21 17:41:10  guettler
 *   - adapted to enhanced configurable interface: console can now handle configurable childs of configurables
 *
 *   Revision 1.10  2010/11/11 08:58:45  martius
 *   comments and testing
 *
 *   Revision 1.9  2010/09/27 14:55:56  martius
 *   reimplemented cmd_set. Works now better, also with spaces after the = sign
 *   line variable contains full cmd line now
 *
 *   Revision 1.8  2010/09/16 10:00:21  martius
 *   storage of playground contours
 *   tab completion for parameters
 *
 *   Revision 1.7  2010/09/07 06:32:57  martius
 *   documentation
 *   import -> include
 *
 *   Revision 1.6  2010/06/28 14:52:52  martius
 *   ctrl+c is hidden
 *
 *   Revision 1.5  2008/09/16 14:40:06  martius
 *   made some char* constant to avoid cast error
 *
 *   Revision 1.4  2007/08/29 15:18:19  martius
 *   index starts at 0 for objects
 *
 *   Revision 1.3  2007/04/13 13:10:28  robot4
 *   exceptions handled
 *
 *   Revision 1.2  2007/03/26 13:36:58  martius
 *   typo
 *
 *   Revision 1.1  2007/03/26 13:05:51  martius
 *   new commandline interface
 *
 *                                                                         *
 ***************************************************************************/
 

#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <readline/readline.h>
#include <readline/history.h>

#include <vector>
#include <sstream>
#include <string>
#include <selforg/stl_adds.h>
#include <selforg/abstractcontroller.h>
#include "globaldata.h"
#include "odeagent.h"
#include "abstractground.h"

using namespace std;

namespace lpzrobots {


typedef bool (*commandfunc_t)(GlobalData& globalData, char *, char *);          
/* The names of functions that actually do the manipulation.  parameter: global data, entire line, arg */
bool com_list (GlobalData& globalData, char *, char *);
bool com_show (GlobalData& globalData, char *, char *);
bool com_store (GlobalData& globalData, char *, char *);
bool com_load (GlobalData& globalData, char *, char *);
bool com_storecfg (GlobalData& globalData, char *, char *);
bool com_loadcfg (GlobalData& globalData, char *, char *);
bool com_contrs (GlobalData& globalData, char *, char *);
bool com_set (GlobalData& globalData, char *, char *);
bool com_help (GlobalData& globalData, char *, char *);
bool com_quit (GlobalData& globalData, char *, char *);

/* A structure which contains information on the commands this program
   can understand. */

typedef struct {
  const char *name;                   /* User printable name of the function. */
  commandfunc_t func;           /* Function to call to do the job. */
  const char *doc;                    /* Documentation for this function.  */
} COMMAND;

COMMAND commands[] = {
  { "param=val",  com_set, "sets PARAM of all objects to VAL" },
  { "help", com_help, "Display this text" },
  { "?",     com_help, "Synonym for `help'" },
  { "list", com_list, "Lists all configurables and agents" },
  { "ls",   com_list, "Synonym for `list'" },
  { "set",  com_set, "syntax: set [OBJECTID] PARAM=VAL; sets parameter of OBJECTID (or of all objects) to value" },
  { "store", com_store, "stores object. Syntax: store AGENTID FILE, see list" },
  { "load", com_load, "loads object. Syntax: load AGENTID FILE, see list" },
  { "storecfg", com_storecfg, "Store key-values pairs. Syntax: storecfg CONFIGID FILE" },
  { "loadcfg", com_loadcfg, "Load key-values pairs. Syntax: CONFIGID FILE" },
  { "contrs", com_contrs, "Stores the contours of all playgrounds to FILE" },
  { "show", com_show, "[OBJECTID]: Lists parameters of OBJECTID or of all objects (if no id given)" },
  { "view", com_show, "Synonym for `show'" },
  { "quit", com_quit, "Quit program" },
  { (char *)NULL, (commandfunc_t)NULL, (char *)NULL }
};

  typedef std::list<std::string> ParameterList;
ParameterList parameters; // used for completion

/* Forward declarations. */
char * stripwhite (char *string);
COMMAND *find_command (char *name);
bool execute_line (GlobalData& globalData, char *line);
int valid_argument ( const char *caller, const char *arg); 


void printConfigs(const ConfigList& configs)
{

  struct winsize w;
  ioctl(0, TIOCGWINSZ, &w);
  FOREACHC(ConfigList, configs, c){
    (*c)->print(stdout, 0, w.ws_col-2, true);
  }
}

void printConfig(const Configurable* config)
{
  struct winsize w;
  ioctl(0, TIOCGWINSZ, &w);
  if(config) config->print(stdout, 0, w.ws_col-2, true);
}



char* dupstr (const char* s){
  char *r;

  r = (char*)malloc (strlen (s) + 1);
  strcpy (r, s);
  return (r);
}

// duplicates string and adde an = sign
char* dupstrpluseq (const char* s){
  char *r;
  int len = strlen (s);
  r = (char*)malloc (strlen (s) + 2);
  strcpy (r, s);
  r[len]= '=';
  r[len+1]= 0;
  return (r);
}

vector<string> splitstring(string s){
  istringstream split(s); //splitting the lines
  string word;
  vector<string> rv;
  while(split >> word) {
    if(word.compare(" ")!=0 && word.compare("  ")!=0)
      rv.push_back(word);
  }
  return rv;
}

bool handleConsole(GlobalData& globalData){
  char *line, *s;
  bool rv = true;
  
  //  initialize_readline ();       /* Bind our completer. */
  // move to beginning of line (clear the ^C)
  std::cout << "\033[1G" << "Type: ? for help or press TAB\n";
  // collect parameters for completion
  parameters.clear();
  for(vector<Configurable*>::const_iterator i=globalData.configs.begin(); i != globalData.configs.end(); i++){
    if(*i)
      parameters += (*i)->getAllParamNames();
  }

  line = readline ("> ");
  
  if (!line)
    return rv;
  
  /* Remove leading and trailing whitespace from the line.
     Then, if there is anything left, add it to the history list
     and execute it. */
  s = stripwhite (line);  
  if (*s) {    
    add_history (s);
    rv = execute_line (globalData,s);
  }
  
  free (line);
  return rv;
}

/* Execute a command line. */
bool execute_line (GlobalData& globalData, char *line) {
  register int i;
  COMMAND *command;
  char *word;

  /* Isolate the command word. */
  i = 0;
  while (line[i] && whitespace (line[i]))
    i++;
  word = line + i;

  while (line[i] && !whitespace (line[i]))
    i++;

  bool args = line[i] != 0;
  if (args)
    line[i] = '\0';

  command = find_command (word);
  
  if(args)
    line[i++] = ' '; // reinsert space
  
  if (!command)
    {
      fprintf (stderr, "%s: No such command\n", word);
      return (-1);
    }

  /* Get argument to command, if any. */
  while (whitespace (line[i]))
    i++;

  word = line + i;

  /* Call the function. */
  return ((*(command->func)) (globalData, line, word));
}

/* Look up NAME as the name of a command, and return a pointer to that
   command.  Return a NULL pointer if NAME isn't a command name. */
COMMAND *find_command (char *name){
  register int i;
  char *p = strchr(name,'=');
  if(p) return (&commands[0]); // set a parameter.
  for (i = 0; commands[i].name; i++)
    if (strcmp (name, commands[i].name) == 0)
      return (&commands[i]);

  return ((COMMAND *)NULL);
}

/* Strip whitespace from the start and end of STRING.  Return a pointer
   into STRING. */
char * stripwhite (char *string){
  register char *s, *t;

  for (s = string; whitespace (*s); s++)
    ;
    
  if (*s == 0)
    return (s);

  t = s + strlen (s) - 1;
  while (t > s && whitespace (*t))
    t--;
  *++t = '\0';

  return s;
}

/* **************************************************************** */
/*                                                                  */
/*                  Interface to Readline Completion                */
/*                                                                  */
/* **************************************************************** */

char *command_generator (const char *, int);
char *params_generator (const char *, int);
//char **console_completion __P((const char *, int, int));
char **console_completion (const char *, int, int);

/* Tell the GNU Readline library how to complete.  We want to try to
   complete on command names if this is the first word in the line, or
   on filenames if not. */
void initializeConsole ()
{
  /* Allow conditional parsing of the ~/.inputrc file. */
  rl_readline_name = "LPZRobots_Console";

  /* Tell the completer that we want a crack first. */
  rl_attempted_completion_function = console_completion;

  read_history (".history");
}

// store the history
void closeConsole(){
  write_history(".history");
}

int getListLen(char **strings){
  int i=0;
  if(!strings) return 0;
  while ( strings[i] ){
    i++;
  }
  return i;
}

/* Attempt to complete on the contents of TEXT.  START and END
   bound the region of rl_line_buffer that contains the word to
   complete.  TEXT is the word to complete.  We can use the entire
   contents of rl_line_buffer in case we want to do some simple
   parsing.  Return the array of matches, or NULL if there aren't any. */
char ** console_completion (const char *text, int start, int end) {

  char **matchesCmd = (char **)NULL;
  char **matchesParams = (char **)NULL;

  /* If this word is at the start of the line, then it is a command
     to complete or a parameter name.
     Otherwise it is the name of a file in the current
     directory. 
     if "set" is at the start then it is also a parameter.
  */
  try{
    if (start == 0){
      matchesCmd = rl_completion_matches (text, command_generator);
    }
    if(start==0 || (strncmp(rl_line_buffer,"set",3)==0)){
      matchesParams = rl_completion_matches (text, params_generator);        
    }    
  }catch(...){}
  // merge them;
  int lCmd=getListLen(matchesCmd);
  int lPar=getListLen(matchesParams);
  if(lCmd+lPar > 0){
    char **matches = (char **)malloc((lCmd+lPar+1)*sizeof(char*));
    memcpy(matches,matchesCmd,sizeof(char*)*lCmd);
    memcpy(matches+lCmd,matchesParams,sizeof(char*)*lPar);
    matches[lCmd+lPar]=(char *)NULL;
    return matches;
  } else  
    return (char **)NULL;
  

}

/* Generator function for command completion.  STATE lets us
   know whether to start from scratch; without any state
   (i.e. STATE == 0), then we start at the top of the list. */
char * command_generator (const char *text, int state) {
  static int list_index, len;
  const char *name;

  /* If this is a new word to complete, initialize now.  This
     includes saving the length of TEXT for efficiency, and
     initializing the index variable to 0. */
  if (!state)
    {
      list_index = 1;
      len = strlen (text);
    }

  /* Return the next name which partially matches from the
     command list. */
  while ( (name = commands[list_index].name) )
    {
      list_index++;

      if (strncmp (name, text, len) == 0)
        return (dupstr(name));
    }  

  /* If no names matched, then return NULL. */
  return ((char *)NULL);
}

/* Generator function for parameter completion.  STATE lets us
   know whether to start from scratch; without any state
   (i.e. STATE == 0), then we start at the top of the list. */
char * params_generator (const char *text, int state) {
  static int len;
  static ParameterList::iterator list_it;

  /* If this is a new word to complete, initialize now.  This
     includes saving the length of TEXT for efficiency, and
     initializing the index variable to 0. */
  if (!state)
    {
      list_it = parameters.begin();
      len = strlen (text);
    }

  /* Return the next name which partially matches from the
     parameter list. */
  while ( list_it != parameters.end())
    {      
      if (list_it->find(text, 0, len) == 0){
        char* name = dupstrpluseq(list_it->c_str());
        list_it++;
        return name;
      }
      list_it++;
    }  

  /* If no names matched, then return NULL. */
  return ((char *)NULL);
}


/* **************************************************************** */
/*                                                                  */
/*                       Console Commands                           */
/*                                                                  */
/* **************************************************************** */


bool com_list (GlobalData& globalData, char* line, char* arg) {
  int i=1;
  printf("Agents -------------(for store and load)\nID: Name\n");
  
  FOREACHC(OdeAgentList, globalData.agents,a){    
    if((*a)->getRobot())
    printf(" %3i: %s (Controller and Robot)\n", i, (*a)->getRobot()->getName().c_str());
    printf(" %3i:  |- only Controller\n", i*100+1);
    printf(" %3i:  |- only Robot\n", i*100+2);    
    i++;
  }
  printf("Configurables ------(for set, show, storecfg and loadcfg )\nID: Name\n");
  i=1;
  FOREACHC(ConfigList, globalData.configs,c){    
    printf(" %2i: %s\n", i, (*c)->getName().c_str());
    i++;
  }
  return true;
}

bool com_show (GlobalData& globalData, char* line, char* arg) {
  if (arg && *arg){
    int id = atoi(arg);
    if(id>=0 && id < (signed)globalData.configs.size()){
      printConfig(globalData.configs[id]);
      return true;
    }
  }
  printConfigs(globalData.configs);
 
  return true;
}

bool com_set (GlobalData& globalData, char* line, char* arg) {  
  if(strstr(line,"set")!=line) arg=line; // if it is not invoked with set then it was param=val
  if (valid_argument("set", arg)){
    /* Isolate the command word. */
    bool changed=false;
      
    char * equalpos = strchr(arg,'=');
    if(equalpos) {
      *equalpos=' '; // replace by space for splitting
    }else{
      printf("Syntax error! no '=' found, see help\n");
    }
    vector<string> params = splitstring(string(arg));
    switch(params.size()){
    case 3:// ObjectID param=val      
      {
        int id = atoi(params[0].c_str());
        if(id>=0 && id < (signed)globalData.configs.size()){
          const char* key = params[1].c_str();        
          if (globalData.configs[id]->setParam(key,atof(params[2].c_str()))){
            printf(" %s=\t%f\t%s\n", key, globalData.configs[id]->getParam(key),
                   globalData.configs[id]->getName().c_str());
            changed = true;              
          }          
        }else printf("Object with ID: %i not found\n", id);      
      }
      break;
    case 2: // param=val
      {
        double v=atof(params[1].c_str());
        const char* key = params[0].c_str();
 	FOREACH(ConfigList, globalData.configs, i){
 	  if ((*i)->setParam(key,v)){
 	    printf(" %s=\t%f\t%s\n", key, (*i)->getParam(key), (*i)->getName().c_str());
 	    changed = true;	    
 	  }
 	}
      }
      break;
    default: // something else
      printf("Syntax Error! Expect 2 or 3 arguments: [ObjectID] param=val\n");
      printf("Got %i params:", (int)params.size());
      FOREACHC(vector<string>, params, p) printf("%s, ", p->c_str());
      printf("\n");
      break;
    }
    if(changed){
      *equalpos='=';
      FOREACH(OdeAgentList, globalData.agents, i){	
	(*i)->writePlotComment(arg);
      }      
    }
    
      
    // }elseif(params.size(==2) )
    // s_param = strchr(arg,' ');
    // if(s_param) *s_param='\0'; // terminate first arg
    // if(s_param && strchr(arg,'=')==NULL){ // looks like two args (and no = in the first)
    //   s_param++;
    //   int id = atoi(arg);
    //   if(id>=0 && id < (signed)globalData.configs.size()){
    //     char* val;
    //     i=0;
    //     val = strchr(s_param,'=');
    //     if(val){ // found =
    //       *val='\0';
    //       double v=strtod(val+1,0);
    //       if (globalData.configs[id]->setParam(s_param,v)){
    //         printf(" %s=\t%f \n", s_param, globalData.configs[id]->getParam(s_param));
    //         changed = true;
    //         *val='='; // remove termination again (for agent notification)
    //       }
    //     } else printf("Syntax error! no '=' found\n");      
    //   }else printf("Object with ID: %i not found\n", id);      
    // }else{
    //   if(s_param) *s_param=' '; // unterminate arg
    //   s_param=arg;
	
    //   char* val;
    //   i=0;
    //   val = strchr(s_param,'=');
    //   if(val){ // found =
    //     *val='\0';
    //     double v=strtod(val+1,0);
    //     FOREACH(ConfigList, globalData.configs, i){
    //       if ((*i)->setParam(s_param,v)){
    //         printf(" %s=\t%f \n", s_param, (*i)->getParam(s_param));
    //         changed = true;	    
    //       }
    //     }
    //     *val='='; // remove termination again (for agent notification)
    //   } else printf("Syntax error! no '=' found\n");      
    // }
    // if(changed){
    //   FOREACH(OdeAgentList, globalData.agents, i){	
    //     (*i)->writePlotComment(s_param );
    //   }      
    // }
  }
  return true;
}

bool com_store (GlobalData& globalData, char* line, char* arg) {
  if (valid_argument("store", arg)){
    char* filename;        
    short sub=0;
    filename = strchr(arg,' ');
    if(filename) { // we have 2 arguments
      *filename='\0';
      filename++;
      int id = atoi(arg);
      if(id>=100) {
	sub=id%100;
	id=id/100;
      }
      if(id>=1 && id <= (signed)globalData.agents.size()){
	FILE* f = fopen(filename,"wb");
	if(f){
	  switch(sub){
	  case 0: // store agent
	    if(globalData.agents[id-1]->store(f))
	      printf("Agent stored\n");
	    else printf("Error occured while storing agent\n");
	    break;	    
	  case 1: // store controller
	    if(globalData.agents[id-1]->getController()->store(f))
	      printf("Controller stored\n");
	    else printf("Error occured while storing contoller\n");
	    break;
	  case 2: // store robot
	    if(globalData.agents[id-1]->getRobot()->store(f))
	      printf("Robot stored\n");
	    else printf("Error occured while storing robot\n");
	    break;
	  }
	  fclose(f);
	}else printf("Cannot open file %s for writing\n", filename);
      } else printf("Agent with ID: %i not found\n", id);            
    }else printf("syntax error , see >help store\n");        
  }
  return true;
}

bool com_load (GlobalData& globalData, char* line, char* arg) {
  if (valid_argument("load", arg)){
    char* filename;        
    short sub=0;
    filename = strchr(arg,' ');
    if(filename) { // we have 2 arguments
      *filename='\0';
      filename++;
      int id = atoi(arg);
      if(id>=100) {
	sub=id%100;
	id=id/100;
      }
      if(id>=1 && id <= (signed)globalData.agents.size()){
	FILE* f = fopen(filename,"rb");
	if(f){
	  switch(sub){
	  case 0: // store agent
	    if(globalData.agents[id-1]->restore(f))
	      printf("Agent restored\n");
	    else printf("Error occured while restoring agent\n");
	    break;	    
	  case 1: // store controller
	    if(globalData.agents[id-1]->getController()->restore(f))
	      printf("Controller restored\n");
	    else printf("Error occured while restoring contoller\n");
	    break;
	  case 2: // store robot
	    if(globalData.agents[id-1]->getRobot()->restore(f))
	      printf("Robot restored\n");
	    else printf("Error occured while restoring robot\n");
	    break;
	  }	 
	  fclose(f);
	}else printf("Cannot open file %s for reading\n", filename);
      } else printf("Agent with ID: %i not found\n", id);            
    }else printf("syntax error , see >help load\n");        
  }
  return true;
}

bool com_storecfg (GlobalData& globalData, char* line, char* arg) {
  if (valid_argument("storecfg", arg)){
    char* filename;        
    filename = strchr(arg,' ');
    if(filename) { // we have 2 arguments
      *filename='\0';
      filename++;
      int id = atoi(arg);
      if(id>=1 && id <= (signed)globalData.configs.size()){
	if(globalData.configs[id-1]->storeCfg(filename))
	  printf("Configuration stored\n");
	else 
	  printf("Error occured while storing configuration\n");	
      } else printf("Configurable with ID: %i not found\n", id);            
    }else printf("syntax error , see >help storecfg\n");        
  }
  return true;
}

bool com_loadcfg (GlobalData& globalData, char* line, char* arg) {
  if (valid_argument("loadcfg", arg)){
    char* filename;        
    filename = strchr(arg,' ');
    if(filename) { // we have 2 arguments
      *filename='\0';
      filename++;
      int id = atoi(arg);
      if(id>=1 && id <= (signed)globalData.configs.size()){
	if(globalData.configs[id-1]->restoreCfg(filename))
	  printf("Configuration restored\n");
	else 
	  printf("Error occured while restoring configuration\n");
      } else printf("Configurable with ID: %i not found\n", id);            
    }else printf("syntax error , see >help loadcfg\n");        
  }
  return true;
}

bool com_contrs (GlobalData& globalData, char* line, char* arg) {
  if (valid_argument("contours", arg)){
    char* filename;        
    filename = arg;
    if(filename) { // we at least 1 argument
      FILE* f = fopen(filename,"wb");
      if(f){
        int i=0;
        FOREACHC(ObstacleList, globalData.obstacles, o) {
          AbstractGround* g = dynamic_cast<AbstractGround*>(*o);
          if(g){
            fprintf(f, "# Contour from Playground %i\n", i);
            g->printContours(f);
            i++;
          }
        }
        printf("%i playground contours saved to %s\n", i, filename);
        fclose(f);
      }else printf("Cannot open file %s for writing\n", filename);              
    }else printf("syntax error , see >help store\n");        
  }
  return true;
}


bool com_quit (GlobalData& globalData, char *, char *){
  return false;
}

/* Print out help for ARG, or for all of the commands if ARG is
   not present. */
bool com_help (GlobalData& globalData, char* line, char* arg) {
  register int i;
  int printed = 0;

  for (i = 0; commands[i].name; i++)
    {
      if (!*arg || (strcmp (arg, commands[i].name) == 0))
        {
          printf (" %s\t\t%s.\n", commands[i].name, commands[i].doc);
          printed++;
        }
    }

  if (!printed)
    {
      printf ("No commands match `%s'.  Possibilties are:\n", arg);

      for (i = 0; commands[i].name; i++)
        {
          /* Print in six columns. */
          if (printed == 6)
            {
              printed = 0;
              printf ("\n");
            }

          printf (" %s\t", commands[i].name);
          printed++;
        }

      if (printed)
        printf ("\n");
    }
  return true;
}



/* Return non-zero if ARG is a valid argument for CALLER,
   else print an error message and return zero. */
int
valid_argument ( const char *caller, const char *arg)
{
  if (!arg || !*arg)
    {
      fprintf (stderr, "%s: Argument required.\n", caller);
      return (0);
    }

  return (1);
}

}
