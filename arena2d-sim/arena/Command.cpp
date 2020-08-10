#include "Command.h"

void CommandTools::splitCommand_free(char ** c, int size)
{
	for(int i = 0; i < size; i++)
		delete[](c[i]);
	
	delete[] c;
}

/* helper function to add a token */
char* CommandTools::splitCommand_add(const char * s, int start, int end)
{
	int size = end - start + 1;
	if(size < 0)
		return NULL;
	char * t = new char[size+1];
	memcpy(t, s+start, size);
	t[size] = '\0';
	return t;
}

int CommandTools::parseCommand(int argc, const char * argv[], CommandArgument * cmd_args, int num_cmd_args)
{
	int start = 0;
	while(start < argc)
	{
		if(argv[start][0] == '-')/* argument with flag */
		{
			/* find corresponding Command argument */
			bool found = false;
			for(int i = 0; i < num_cmd_args; i++)
			{
				if(cmd_args[i].flag != NULL && strcmp(cmd_args[i].flag, &argv[start][1]) == 0)
				{
					if(cmd_args[i].was_set)
					{
						ERROR_F("Multiple occurence of parameter: %s", argv[start]);
						return -1;
					}
					if(cmd_args[i].type == CommandArgument::BOOL)
					{
						cmd_args[i].setValue(argv[start]);
						start++;
						found = true;
						break;
					}
					else
					{
						if(start+1 < argc)/* there is a value */
						{
							cmd_args[i].setValue(argv[start+1]);
							start += 2;
							found = true;
							break;
						}
						else
						{
							ERROR_F("Parameter requires value: %s", argv[start]);
							return -1;
						}
					}
				}
			}
			if(!found)
			{
				/* not found */
				ERROR_F("No such parameter: %s", argv[start]);
				return -1;/* error no such argument */
			}
		}
		else /* argument without flag */
		{
			bool found = false;
			for(int i = 0; i < num_cmd_args; i++)
			{
				if(cmd_args[i].flag == NULL)/* empty flag */
				{
					cmd_args[i].setValue(argv[start]);
					start++;
					found = true;
					break;
				}
			}
			if(!found)
			{
				/* not found */
				ERROR_F("Argument without Flag: %s", argv[start]);
				return -1;
			}
		}
	}
	/* check whether all required arguments have been set */
	for(int i = 0; i < num_cmd_args; i++)
	{
		if(cmd_args[i].required && !cmd_args[i].was_set)/* required but not set */
		{
			if(cmd_args[i].flag != NULL)
				ERROR_F("Required parameter not set %s", cmd_args[i].flag);
			else
				ERROR("Required parameter not set");
			return  -1;
		}
	}
	return 0;
}

const char * CommandTools::splitCommand(const char * s, int * num_tokens, char *** tokens)
{
	list<char*> token_list;
	bool quote_marks = false; /* quotation marks have been found */
	bool last_white = true; /* last character was separation */
	int start = 0;
	int i = 0;
	bool add_token;
	/* skip all whitespaces at the beginning */
	while(s[i] != '\0' && s[i] != ';')
	{
		add_token = false;
		if(!quote_marks)
		{
			if(s[i] == ' ')
			{
				if(last_white)
				{
				}
				else
				{
					token_list.push_back(splitCommand_add(s, start, i-1));
				}
				start = i+1;
				last_white = true;
			}
			else if(s[i] == '\"')
			{
				if(!last_white){
					token_list.push_back(splitCommand_add(s, start, i-1));
				}
				last_white = true;
				quote_marks = true;
				start = i+1;
			}
			else
			{
				last_white = false;
			}
		}
		else/* inside a string literal */
		{
			if(s[i] == '\"')
			{
				quote_marks = false;
				token_list.push_back(splitCommand_add(s, start, i-1));
				start = i+1;
				last_white = true;
			}
			else
				last_white = false;
		}
		i++;
	}
	/* add last token */
	if(i != start)
	{
		char * t = splitCommand_add(s, start, i-1);
		token_list.push_back(t);
	}
	if(s[i] == ';')
		i++;

	/* create new array of strings - add all element from list into that array */
	char ** token_array = new char*[token_list.size()];
	list<char*>::iterator it = token_list.begin();
	int count = 0;
	while(it != token_list.end())
	{
		token_array[count] = *it;
		it++;
		count++;
	}
	*num_tokens = token_list.size();

	*tokens = token_array;
	return s+i;
}
