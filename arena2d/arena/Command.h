/*
 * Command.h
 *
 *  Created on: May 20, 2018
 *      Author: zer0divider
 */

#ifndef COMMAND_H_
#define COMMAND_H_

#include <string>
#include <unordered_map>
#include <string.h>
#include <list>
#include <assert.h>
#include <engine/zLogfile.h>
#include "CommandStatus.h"

using namespace std;

struct CommandArgument{
	enum Type{BOOL, INT, FLOAT, STRING};
	CommandArgument(const char * _flag, double * v, bool _required = false): required(_required), flag(_flag), type(FLOAT), value(v), was_set(false) {}
	CommandArgument(const char * _flag, bool * v, bool _required = false): required(_required),flag(_flag), type(BOOL), value(v), was_set(false){*((bool*)v) = false;}
	/* bool argument is default false and will be set to true on a call to setValue() (ignoring the argument) */
	CommandArgument(const char * _flag, int * v, bool _required = false): required(_required),flag(_flag), type(INT), value(v), was_set(false){}
	CommandArgument(const char * _flag, const char ** v, bool _required = false): required(_required),flag(_flag), type(STRING), value(v), was_set(false){}
	~CommandArgument(){}
	void setValue(const char * arg){
		switch(type)
		{
			case BOOL:{*((bool*)value) = true;}break;
			case INT:{*((int*)value) = atoi(arg);}break;
			case FLOAT:{*((double*)value) = atof(arg);}break;
			case STRING:{*((const char**)value) = arg;}break;
			default:{assert(0);}/* should not happen */
		}
		was_set = true;
	}
	void * value;
	const char * flag;/* corresponding flag that needs to be set (e.g. -port),
				if set to NULL, value that doesn't have a flag (<value>) will be considered */
	bool was_set;/* value was set */
	bool required;/* argument needs to be set */
	Type type;
};

/* helper functions */
namespace CommandTools {

	/* splitting given text into arguments separated by ' ' until ';' is read
		number of tokens is stored in given ptr <num_tokens>
		@tokens contains an array of strings with <num_tokens> elements containing separate tokens
		return: pointer to the next command inside given string (commands are separated by ';')
	*/
	const char * splitCommand(const char * t, int * num_tokens, char *** tokens);
	 
	 /*split command helper */
	char* splitCommand_add(const char * s, int start, int end);

	/* return value of splitCommand can be passed to free the array
		<size> is the number of elements in that array
	*/
	void splitCommand_free(char ** c, int size);

	/* parses given command arguments from a list of Command arguments
		returns -1 on error, else 0
	*/
	int parseCommand(int argc, const char * argv[], CommandArgument * cmd_args, int num_cmd_args);

};

/* command argument represents an console argument that is parsed according to its type */

/* name, description and hints of a command */
struct CommandDescription
{
	CommandDescription(){}
	CommandDescription(const string & _name, const string & _hint, const string & _descr):
	name(_name), hint(_hint), descr(_descr){}

	string name;  /* command name */
	string hint;  /* required (and optional) parameters */
	string descr; /* what does the command do? */
};

/* abstract class providing functions for creating custom commands that are executed during application runtime */
template<class T>
class DevCommand
{
public:
	/* pointer to a member function of T */
	typedef CommandStatus (T::*cmd_p)(int, const char**);

	/* constructor: create a new command with given attributes
		name: name of the command
		hint: list of the parameters used in this command
		descr: description of what the command does */
	DevCommand(cmd_p command, const string& name, const string& hint = "", const string& descr = ""):
	_descriptor(name, hint, descr){
		_command =  command;
		}
	
	/* destructor */
	virtual ~DevCommand(){}
	
	/* execute the command with given arguments (abstract, needs to be overwritten by child-class)
		argc: argument-count, number of given arguments in <argv>
		argv: argument-values, string array containing <argc> elements 
		return: 0 on success */
	CommandStatus exec(T * t, int argc, const char * argv[]){return (t->*_command)(argc, argv);}

	/* getting string members */
	const CommandDescription& getDescription(){return _descriptor;} 
private:
	CommandDescription _descriptor;
	cmd_p _command;
};

#endif /* COMMAND_H_ */
