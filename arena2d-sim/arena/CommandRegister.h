#ifndef COMMAND_REGISTER_H
#define COMMAND_REGISTER_H

#include "Command.h"
#include <engine/zLogfile.h>

/* controlling many commands belonging to one context */
template<class T>
class CommandRegister
{
public:
	/* useful type definitions */
	typedef DevCommand<T> CommandType;
	typedef unordered_map<string, CommandType*> HashType;
	typedef typename HashType::value_type ValuePair;

	/* constructor */
	CommandRegister()/*:_lastCommand(NULL)*/{}

	/* destructor */
	~CommandRegister();


	/* register a new command */
	void registerCommand(typename DevCommand<T>::cmd_p func, const string& name,
													const string& hint,
													const string& descr);

	/* execute a command by given name and parameters */
	CommandStatus execCommand(T * t, const string & name, int argc, const char * argv[]); 
	
	/* get command that corresponds to the given name
		return NULL if command was not found */
	DevCommand<T>* getCommand(const string & name);

	/* get last error that occured */
	const char* getError();

	/* getting all command descriptions that are currently in the register */
	void getAllCommandDescriptions(list<const CommandDescription*> & cmd_list);
private:
	DevCommand<T> * _lastCommand; /* last command that was executed */
	HashType _hashTable;

};


template<class T>
CommandRegister<T>::~CommandRegister()
{
	/* remove all instanciated dev commands */
	typename HashType::iterator it = _hashTable.begin();	
	while(it != _hashTable.end())
	{
		delete(it->second);
		it++;
	}
}

template<class T>
void CommandRegister<T>::registerCommand(typename DevCommand<T>::cmd_p func, const string& name,
																 const string& hint,
																 const string& descr)
{
	pair<typename HashType::iterator, bool> res = _hashTable.insert(ValuePair(name, new DevCommand<T>(func, name, hint, descr)));
	if(!res.second)/* command already exists */
	{
		WARNING_F("Command has already been added: %s", name.c_str());
	}
}


template<class T>
CommandStatus CommandRegister<T>::execCommand(T * t, const string& name, int argc, const char * argv[])
{
	/* trying to find command */
	DevCommand<T> * cmd = getCommand(name);
	if(cmd == NULL)/* command not found */
		return CommandStatus::UNKNOWN_COMMAND;

	/* command has been found -> executing */
	_lastCommand = cmd;
	return cmd->exec(t, argc, argv);
}

template<class T>
DevCommand<T>* CommandRegister<T>::getCommand(const string & name)
{
	/* find command by name */
	typename HashType::iterator it = _hashTable.find(name);

	if(it != _hashTable.end())/* command exists */
		return it->second;
	return NULL; /* command does not exist */
}

template<class T>
void CommandRegister<T>::getAllCommandDescriptions(list<const CommandDescription*> & cmd_list)
{
	/* going through every bucket and fetching each command */
	int i = 0;
	while(i < _hashTable.bucket_count())
	{
		/* iterator over every item in that bucket */
		typename HashType::local_iterator it = _hashTable.begin(i);
		while(it != _hashTable.end(i))
		{
			DevCommand<T> *d = it->second;
			const CommandDescription& descr = d->getDescription();
			cmd_list.push_back(&descr);
			it++;
		}
		i++;
	}
}

#endif
