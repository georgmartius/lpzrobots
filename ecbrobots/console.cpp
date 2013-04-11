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
 *   Revision 1.5  2010-11-10 09:32:00  guettler
 *   - port to Qt part 1
 *
 *   Revision 1.4  2009/08/11 15:49:05  guettler
 *   Current development state:
 *   - Support of communication protocols for XBee Series 1, XBee Series 2 and cable mode
 *   - merged code base from ecb_robots and Wolgang Rabes communication handling;
 *     ECBCommunicator (almost) entirely rewritten: Use of Mediator (MediatorCollegues: ECB),
 *     Callbackble (BackCaller: SerialPortThread)
 *   - New CThread for easy dealing with threads (is using pthreads)
 *   - New TimerThreads for timed event handling
 *   - SerialPortThread now replaces the cserialthread
 *   - QGlobalData, ECBCommunicator is now configurable
 *   - ECBAgent rewritten: new PlotOptionEngine support, adapted to new WiredController structure
 *   - ECBRobot is now Inspectables (uses new infoLines functionality)
 *   - ECB now supports dnsNames and new communication protocol via Mediator
 *   - Better describing command definitions
 *   - SphericalRobotECB: adapted to new ECB structure, to be tested
 *
 *   Revision 1.3  2009/03/25 11:06:55  robot1
 *   updated version
 *
 *   Revision 1.2  2008/04/11 06:31:16  guettler
 *   Included all classes of ecbrobots into the namespace lpzrobots
 *
 *   Revision 1.1.1.1  2008/04/08 08:14:30  guettler
 *   new ecbrobots module!
 *
 *   Revision 1.2  2007/07/28 10:13:26  martius
 *   very nicely working communication with multiple xbees *g*
 *
 *   Revision 1.1  2007/07/10 13:42:00  robot3
 *   added readline console
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
#include <readline/readline.h>
#include <readline/history.h>

#include <vector>
#include <selforg/stl_adds.h>
#include <selforg/abstractcontroller.h>
#include "QGlobalData.h"
#include <selforg/agent.h>
#include <selforg/configurable.h>
#include <selforg/abstractrobot.h>
#include "ecbagent.h"

#include <string.h>

using namespace std;


namespace lpzrobots {


typedef bool (*commandfunc_t)(QGlobalData& globalData, char *, char *);
/* The names of functions that actually do the manipulation.  parameter: global data, entire line, arg */
bool com_list (QGlobalData& globalData, char *, char *);
bool com_show (QGlobalData& globalData, char *, char *);
bool com_store (QGlobalData& globalData, char *, char *);
bool com_load (QGlobalData& globalData, char *, char *);
bool com_set (QGlobalData& globalData, char *, char *);
bool com_help (QGlobalData& globalData, char *, char *);
bool com_quit (QGlobalData& globalData, char *, char *);

/* A structure which contains information on the commands this program
   can understand. */

typedef struct {
  char *name;                   /* User printable name of the function. */
  commandfunc_t func;           /* Function to call to do the job. */
  char *doc;                    /* Documentation for this function.  */
} COMMAND;

COMMAND commands[] = {
  { (char*)string("param=val").c_str(),  com_set, (char*)string("sets PARAM of all objects to VAL").c_str() },
  { (char*)string("help").c_str(), com_help, (char*)string("Display this text").c_str() },
  { (char*)string("?").c_str(),     com_help, (char*)string("Synonym for `help'").c_str() },
  { (char*)string("list").c_str(), com_list, (char*)string("Lists all objects and agents").c_str() },
  { (char*)string("ls").c_str(),   com_list, (char*)string("Synonym for `list'").c_str() },
  { (char*)string("set").c_str(),  com_set, (char*)string("syntax: set [OBJECTID] PARAM=VAL; sets parameter of Object (or all all objects) to value").c_str() },
  { (char*)string("store").c_str(), com_store, (char*)string("Stores controller of AGENTID to FILE").c_str() },
  { (char*)string("load").c_str(), com_load, (char*)string("Loads controller of AGENTID from FILE").c_str() },
  { (char*)string("show").c_str(), com_show, (char*)string("[OBJECTID]: Lists paramters of OBJECTID or of all objects (if no id given)").c_str() },
  { (char*)string("view").c_str(), com_show, (char*)string("Synonym for `show'").c_str() },
  { (char*)string("quit").c_str(), com_quit, (char*)string("Quit program").c_str() },
  { (char *)(NULL), (commandfunc_t)NULL, (char *)NULL }
};

/* Forward declarations. */
char * stripwhite (char *string);
COMMAND *find_command (char *name);
bool execute_line (QGlobalData& globalData, char *line);
int valid_argument ( char *caller, char *arg);
void too_dangerous ( char *caller );


void showParams(const ConfigList& configs)
{
  for(vector<Configurable*>::const_iterator i=configs.begin(); i != configs.end(); i++){
    (*i)->print(stdout, 0);
  }
}

void showParam(const Configurable* config)
{
  if(config) config->print(stdout, 0);
}



char* dupstr (const char* s){
  char *r;

  r = (char*)malloc (strlen (s) + 1);
  strcpy (r, s);
  return (r);
}

bool handleConsole(QGlobalData& globalData){
  char *line, *s;
  bool rv = true;

  //  initialize_readline ();       /* Bind our completer. */
  fflush(stdout);
  std::cout << "Type: ? for help or press TAB\n";
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
bool execute_line (QGlobalData& globalData, char *line) {
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

  if (line[i])
    line[i++] = '\0';

  command = find_command (word);

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
  if(p) return (&commands[0]);
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

/// store the history
void closeConsole(){
  write_history(".history");
}

/* Attempt to complete on the contents of TEXT.  START and END
   bound the region of rl_line_buffer that contains the word to
   complete.  TEXT is the word to complete.  We can use the entire
   contents of rl_line_buffer in case we want to do some simple
   parsing.  Returnthe array of matches, or NULL if there aren't any. */
char ** console_completion (const char *text, int start, int end) {
  char **matches;

  matches = (char **)NULL;

  /* If this word is at the start of the line, then it is a command
     to complete.  Otherwise it is the name of a file in the current
     directory. */
         try{
                if (start == 0)
                    matches = rl_completion_matches (text, command_generator);
        }catch(...){}
  return (matches);
}

/* Generator function for command completion.  STATE lets us
   know whether to start from scratch; without any state
   (i.e. STATE == 0), then we start at the top of the list. */
char * command_generator (const char *text, int state) {
  static int list_index, len;
  char *name;

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

/* **************************************************************** */
/*                                                                  */
/*                       Console Commands                           */
/*                                                                  */
/* **************************************************************** */


bool com_list (QGlobalData& globalData, char* line, char* arg) {
  int i=0;
  printf("Agents ---------------(for store and load)\nID: Name\n");

  FOREACHC(AgentList, globalData.agents,a){
    if((*a)->getRobot())
    printf(" %2i: %s\n", i, (*a)->getRobot()->getName().c_str());
    i++;
  }
  printf("Objects --------------(for set and show)\nID: Name\n");

  FOREACHC(ConfigList, globalData.configs,c){
    printf(" %2i: %s\n", i, (*c)->getName().c_str());
    i++;
  }
  return true;
}

bool com_show (QGlobalData& globalData, char* line, char* arg) {
  if (arg && *arg){
    int id = atoi(arg);
    if(id>=0 && id < (signed)globalData.configs.size()){
      showParam(globalData.configs[id]);
      return true;
    }
  }
  showParams(globalData.configs);

  return true;
}

bool com_set (QGlobalData& globalData, char* line, char* arg) {
  if(strstr(line,"set")!=line) arg=line; // if it is not invoked with set then it was param=val
  if (valid_argument((char*)string("set").c_str(), arg)){
    /* Isolate the command word. */
    int i = 0;
    char* s_param;
    bool changed=false;

    s_param = strchr(arg,' ');
    if(s_param) *s_param='\0'; // terminate first arg
    if(s_param && strchr(arg,'=')==NULL){ // looks like two args (and no = in the first)
      s_param++;
      int id = atoi(arg);
      if(id>=0 && id < (signed)globalData.configs.size()){
        char* val;
        i=0;
        val = strchr(s_param,'=');
        if(val){ // found =
          *val='\0';
          double v=strtod(val+1,0);
          if (globalData.configs[id]->setParam(s_param,v)){
            printf(" %s=\t%f \n", s_param, globalData.configs[id]->getParam(s_param));
            changed = true;
            *val='='; // remove termination again (for agent notification)
          }
        } else printf("Syntax error! no '=' found\n");
      }else printf("Object with ID: %i not found\n", id);
    }else{
      if(s_param) *s_param=' '; // unterminate arg
      s_param=arg;

      char* val;
      i=0;
      val = strchr(s_param,'=');
      if(val){ // found =
        *val='\0';
        double v=strtod(val+1,0);
         FOREACH(ConfigList, globalData.configs, i){
           if ((*i)->setParam(s_param,v)){
             printf(" %s=\t%f \n", s_param, (*i)->getParam(s_param));
             changed = true;
           }
         }
        *val='='; // remove termination again (for agent notification)
      } else printf("Syntax error! no '=' found\n");
    }
    if(changed){
      FOREACH(AgentList, globalData.agents, i){
        (*i)->writePlotComment(s_param );
      }
    }
  }
  return true;
}

bool com_store (QGlobalData& globalData, char* line, char* arg) {
  if (valid_argument((char*)string("store").c_str(), arg)){
    char* filename;
    filename = strchr(arg,' ');
    if(filename) { // we have 2 arguments
      *filename='\0';
      filename++;
      int id = atoi(arg);
      if(id>=0 && id < (signed)globalData.agents.size()){
        FILE* f = fopen(filename,"wb");
        if(f){
          if(globalData.agents[id]->getController()->store(f))
            printf("Controller stored\n");
          else printf("Error occured while storing contoller\n");
          fclose(f);
        }else printf("Cannot open file %s for writing\n", filename);
      } else printf("Agent with ID: %i not found\n", id);
    }else printf("syntax error , see >help store\n");
  }
  return true;
}

bool com_load (QGlobalData& globalData, char* line, char* arg) {
  if (valid_argument((char*)string("load").c_str(), arg)){
    char* filename;
    filename = strchr(arg,' ');
    if(filename) { // we have 2 arguments
      *filename='\0';
      filename++;
      int id = atoi(arg);
      if(id>=0 && id < (signed)globalData.agents.size()){
        FILE* f = fopen(filename,"rb");
        if(f){
          if(globalData.agents[id]->getController()->restore(f))
            printf("Controller restored\n");
          else printf("Error occured while restoring contoller\n");
          fclose(f);
        }else printf("Cannot open file %s for reading\n", filename);
      } else printf("Agent with ID: %i not found\n", id);
    }else printf("syntax error , see >help load\n");
  }
  return true;
}


bool com_quit (QGlobalData& globalData, char *, char *){
  return false;
}

/* Print out help for ARG, or for all of the commands if ARG is
   not present. */
bool com_help (QGlobalData& globalData, char* line, char* arg) {
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
valid_argument ( char *caller, char *arg)
{
  if (!arg || !*arg)
    {
      fprintf (stderr, "%s: Argument required.\n", caller);
      return (0);
    }

  return (1);
}

}
