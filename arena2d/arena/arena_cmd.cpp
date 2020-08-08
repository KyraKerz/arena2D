#include "arena.h"

#ifdef ARENA_USE_PYTHON
const char * PYAGENT_FUNC_NAMES[PYAGENT_FUNC_NUM] = {
	"init",
	"pre_step",
	"post_step",
	"stop"
};
#endif

#define NUM_SHOW_OBJECTS 6
static const char* SHOW_OBJECT_STRINGS[NUM_SHOW_OBJECTS*2] = {
	"stage", "gui.show_stage",
	"robot", "gui.show_robot",
	"laser", "gui.show_laser",
	"stats", "gui.show_stats",
	"goal_spawn", "gui.show_goal_spawn",
	"trail", "gui.show_trail"
};

void Arena::initCommands()
{
	std::string show_hide_objects;
	for(int i = 0; i < NUM_SHOW_OBJECTS; i++){
		show_hide_objects += SHOW_OBJECT_STRINGS[i*2];
		if(i < NUM_SHOW_OBJECTS-1)
			show_hide_objects += ", ";
	}
	/* initialize commands */
	_commands.registerCommand(&Arena::cmdExit, "exit", "", "exit the application");
	_commands.registerCommand(&Arena::cmdShow, "show", "<o1> <o2> ...", std::string("show specified gui objects (") + show_hide_objects + std::string(")"));
	_commands.registerCommand(&Arena::cmdHide, "hide", "<o1> <o2> ...", std::string("hide specified gui objects (") + show_hide_objects + std::string(")"));
	_commands.registerCommand(&Arena::cmdHelp, "help", "[command]", "display information about available commands");
	_commands.registerCommand(&Arena::cmdLoadLevel, "level", "<level_number> [modifier]", "load level with given level number and an optional modifier (='random')");
	_commands.registerCommand(&Arena::cmdReset, "reset", "", "reset current level to initial state");
	_commands.registerCommand(&Arena::cmdSaveSettings, "save_settings", "[filename]", "save current settings to given file, if no filename is specified, the settings are stored to last location");
	_commands.registerCommand(&Arena::cmdLoadSettings, "load_settings", "[filename]", "load current settings from given file, if no filename is specified, the settings are loaded from last location");
	_commands.registerCommand(&Arena::cmdSet, "set", "<settings_expression>", "set an option in the settings, e.g. set \"physics.laser_accuracy = 0.01\"; note that some changes (e.g. video) only take effect after restarting");
	_commands.registerCommand(&Arena::cmdStartTraining, "start_training", "[agent.py [--model <path>] [--device <cpu/cuda>]]", "set in training mode, optionally a python script can be passed with the training routines implemented");
	_commands.registerCommand(&Arena::cmdStartTraining, "run_agent", "[agent.py [--model <path>] [--device <cpu/cuda>]]", "set in training mode, optionally a python script can be passed with the training routines implemented");
	_commands.registerCommand(&Arena::cmdStopTraining, "stop_training", "", "leave training mode");
	_commands.registerCommand(&Arena::cmdSetFPS, "fps", "<video/physics> <fps>", "set frames per second of video or physics thread");
	_commands.registerCommand(&Arena::cmdRealtime, "realtime_factor", "", "set timing so that realtime is accomplished");
}

CommandStatus Arena::cmdRealtime(int argc, const char * argv[])
{
	float factor = 1;
	if(argc > 0)
		factor = atof(argv[0]);
	_SETTINGS->physics.fps = factor/(_SETTINGS->physics.time_step*_SETTINGS->physics.step_iterations);
	return CommandStatus::SUCCESS;
}

CommandStatus Arena::command(const char * c)
{
	CommandStatus s = CommandStatus::SUCCESS;
	while(c != NULL && *c != '\0'){
		int n = 0;
		/* split command */
		char ** tokens;
		c = CommandTools::splitCommand(c, &n, &tokens);

		s = command(tokens[0], n-1, (const char **)(tokens + 1));
		
		/* free tokens */
		CommandTools::splitCommand_free(tokens, n);

		if(s != CommandStatus::SUCCESS)
			break;

	}
	return s;
}

CommandStatus Arena::command(const char * c, int argc, const char * argv[]){
	return _commands.execCommand(this, c, argc, argv);
}

/* get description of a command given by name */
const CommandDescription* Arena::getCommand(const char * cmd){
	auto c = _commands.getCommand(cmd);
	if(c == NULL)
		return NULL;
	
	return &c->getDescription();
}

 /* get all command descriptions */
void Arena::getAllCommands(list<const CommandDescription*> & cmds){
	_commands.getAllCommandDescriptions(cmds);
}

CommandStatus Arena::cmdExit(int argc, const char * argv[])
{
	exitApplication();
	return CommandStatus::SUCCESS;
}

CommandStatus Arena::cmdShow(int argc, const char * argv[]){
	return cmdHideShow(argc, argv, true);
}

CommandStatus Arena::cmdHide(int argc, const char * argv[]){
	return cmdHideShow(argc, argv, false);
}

CommandStatus Arena::cmdHideShow(int num_objects, const char * objects[], bool show)
{
	if(num_objects == 0) return CommandStatus::INVALID_ARG;
	for(int i = 0; i < num_objects; i++){
		std::string expr = "";
		for(int o = 0; o < NUM_SHOW_OBJECTS; o++) {
			if(!strcmp(SHOW_OBJECT_STRINGS[o*2], objects[i])) {
				expr = SHOW_OBJECT_STRINGS[o*2 +1];
				break;
			}
		}
		if(expr.empty())
		{
			ERROR_F("Unknown object name '%s'", objects[i]);
			return CommandStatus::INVALID_ARG;
		}
		if(show)
			expr +="=1";
		else
			expr +="=0";
		_SETTINGS_OBJ->loadFromString(expr.c_str());
	}
	return CommandStatus::SUCCESS;
}

CommandStatus Arena::cmdHelp(int argc, const char * argv[])
{
	if(argc == 0){
		list<const CommandDescription*> cmds;
		getAllCommands(cmds);
		INFO("Commands:");
		for(list<const CommandDescription*>::iterator it = cmds.begin(); it != cmds.end(); it++) {
			INFO_F("\t%s", (*it)->name.c_str());
		}
		INFO("Type 'help <command>' to get information about a specific command.");
	}
	else if(argc > 0){
		const CommandDescription * cmd_desc = getCommand(argv[0]);
		if(cmd_desc == NULL){
			INFO_F("No such command '%s'", argv[0]);
			cmdHelp(0, NULL);
		}
		else{
			INFO_F("%s %s", cmd_desc->name.c_str(), cmd_desc->hint.c_str());
			INFO_F("-> %s", cmd_desc->descr.c_str());
		}
	}
	return CommandStatus::SUCCESS;
}


CommandStatus Arena::cmdLoadLevel(int argc, const char * argv[])
{
	if(_trainingMode){
		showTrainingModeWarning();
		return CommandStatus::EXEC_FAIL;
	}

	int level = atoi(argv[0]);
	bool random = true;
	if(argc > 1){
		if(strcmp(argv[1], "random") == 0)
		{
			random = true;
		}else{
			ERROR_F("Unknown level modifier %s!", argv[1]);
			return CommandStatus::INVALID_ARG;
		}
	}
	if(loadLevel(level, random)){
		return CommandStatus::EXEC_FAIL;
	}else{
		return CommandStatus::SUCCESS;
	}
}

CommandStatus Arena::cmdReset(int argc, const char * argv[])
{
	if(_trainingMode){
		showTrainingModeWarning();
		return CommandStatus::EXEC_FAIL;
	}else{
		reset();
		return CommandStatus::SUCCESS;	
	}
}

CommandStatus Arena::cmdSaveSettings(int argc, const char * argv[])
{
	int ret = 0;
	std::string path;// std::string is used because _lastSettingsPath gets overwritten during saving
	if(argc > 0){
		path = argv[0];
	}
	else{
		path = _SETTINGS_OBJ->getLastSettingsPath();
	}
	ret = _SETTINGS_OBJ->saveToFile(path.c_str());

	if(ret){
		return CommandStatus::EXEC_FAIL;
	}
	else{
		INFO_F("Settings saved to '%s'", path.c_str());
		return CommandStatus::SUCCESS;
	}
}

CommandStatus Arena::cmdLoadSettings(int argc, const char * argv[])
{
	if(_trainingMode){
		showTrainingModeWarning();
		return CommandStatus::EXEC_FAIL;
	}
	int ret = 0;
	std::string path;// std::string is used because _lastSettingsPath gets overwritten during loading
	if(argc > 0){
		path = argv[0];
	}
	else{
		path = _SETTINGS_OBJ->getLastSettingsPath();
	}
	ret = _SETTINGS_OBJ->loadFromFile(path.c_str(), 0);

	if(ret){
		return CommandStatus::EXEC_FAIL;
	}
	else{
		INFO_F("Settings loaded from '%s'", path.c_str());
		return CommandStatus::SUCCESS;
	}
}

CommandStatus Arena::cmdSet(int argc, const char * argv[]){

	if(argc != 1){
		return CommandStatus::INVALID_ARG;
	}

	// laser parameters change?
	float laser_noise = _SETTINGS->robot.laser_noise;
	float laser_max_distance = _SETTINGS->robot.laser_max_distance;
	float laser_start_angle = _SETTINGS->robot.laser_start_angle;
	float laser_end_angle = _SETTINGS->robot.laser_end_angle;
	int laser_num_samples = _SETTINGS->robot.laser_num_samples;

	if(_SETTINGS_OBJ->loadFromString(argv[0])){
		return CommandStatus::EXEC_FAIL;
	}

	// laser parameters change?
	if( laser_noise != _SETTINGS->robot.laser_noise ||
		laser_max_distance != _SETTINGS->robot.laser_max_distance ||
		laser_start_angle != _SETTINGS->robot.laser_start_angle ||
		laser_end_angle != _SETTINGS->robot.laser_end_angle ||
		laser_num_samples != _SETTINGS->robot.laser_num_samples){
		_burger->updateLidar();
		_burger->scan();
	}
	 
	// update environment params
	for(int i = 0; i < _numEnvs; i++)
		_envs[i].refreshSettings();

	return CommandStatus::SUCCESS;
}

CommandStatus Arena::cmdStartTraining(int argc, const char * argv[])
{
	CommandStatus status = CommandStatus::SUCCESS;
	if(!_trainingMode){
		#ifndef ARENA_USE_PYTHON
		if(argc > 0)
			WARNING("Python scripts can't be run! Arena was not compiled with python!");
		#else
		if(argc > 0) // python module given
		{
			_pyAgentUsed = true;
			
			// running python script
			INFO("Opening file...");
			FILE * file = fopen(argv[0], "rb");
			if(file == NULL){
				status = CommandStatus::EXEC_FAIL;
				ERROR_F("Failed to open file '%s': %s", argv[0], strerror(errno));
			}
			else{
				// check for additional parameter
				bool parameter_ok = true;
				int model_index = -1;
				int device_index = -1;
				bool continue_training = false;
				int arg_i = 1;
				while(arg_i < argc){
					if(!strcmp(argv[arg_i], "--model")){
						if(arg_i+1 >= argc){
							ERROR("No model specified!");
							status = CommandStatus::INVALID_ARG;
							parameter_ok = false;
							break;
						}
						model_index = arg_i+1;
						arg_i+= 2;
					}
					else if(!strcmp(argv[arg_i], "--device")){
						if(arg_i+1 >= argc){
							ERROR("No device specified!");
							status = CommandStatus::INVALID_ARG;
							parameter_ok = false;
							break;
						}
						device_index = arg_i+1;
						arg_i+= 2;
					}
					else if(!strcmp(argv[arg_i], "--continue")){
						continue_training = true;
						arg_i+= 1;
					}
					else{
						ERROR_F("Unknown parameter '%s'!", argv[arg_i]);
						status = CommandStatus::INVALID_ARG;
						parameter_ok = false;
						break;
					}
				}
				if(parameter_ok){
					// set argv (absolute path of script that is beeing executed)
					char buffer[256];	
					buffer[0] = '\0';
					getcwd(buffer, 256);
					int cwd_len = strlen(buffer);
					buffer[cwd_len] = '/';
					strcpy(&buffer[cwd_len+1], argv[0]);
					cwd_len = strlen(buffer);
					INFO_F("Executing File under path '%s'...", buffer);
					#ifdef ARENA_PYTHON_VERSION_3
						PyMem_RawFree(_pyArg0);
						_pyArg0 = Py_DecodeLocale(buffer, NULL);
					#else
						delete[](_pyArg0);
						_pyArg0 = new char[cwd_len+1];
						strcpy(_pyArg0, buffer);
					#endif
					PySys_SetArgv(1, &_pyArg0);
					if(PyRun_SimpleFileEx(file, argv[0], 1)){
						status = CommandStatus::EXEC_FAIL;
						ERROR_F("Failed to execute script '%s'!", argv[0]);
					}
					else{
						INFO("Done!");
						_agentModule = PyImport_ImportModule("__main__");
						// getting functions
						for(int i = 0; i < PYAGENT_FUNC_NUM; i++){
							_agentFuncs[i] = PyObject_GetAttrString(_agentModule, PYAGENT_FUNC_NAMES[i]);
							if(_agentFuncs[i] && PyCallable_Check(_agentFuncs[i])){
							}
							else{
								if(_agentFuncs[i] == NULL)
									PyErr_Print();
								WARNING_F("Function '%s' was not found in agent module '%s'!", PYAGENT_FUNC_NAMES[i], argv[0]);
							}
						}

						// call init function
						INFO("Calling init function...");
						PyObject * f = _agentFuncs[PYAGENT_FUNC_INIT];
						if(f){

							PyObject * init_args = PyTuple_New(3);
							// setting parameter
							if(device_index >= 0)
								PyTuple_SetItem(init_args, 0, PyUnicode_FromString(argv[device_index]));
							else
								PyTuple_SetItem(init_args, 0, PyUnicode_FromString("cpu"));

							if(model_index >= 0)
								PyTuple_SetItem(init_args, 1, PyUnicode_FromString(argv[model_index]));
							else
								PyTuple_SetItem(init_args, 1, Py_None);

							PyTuple_SetItem(init_args, 2, PyLong_FromLong(_numEnvs));


							PyObject * result = PyObject_CallObject(f, init_args);
							Py_DECREF(init_args);
							if(result == NULL){
								PyErr_Print();
								ERROR_F("Call to Function '%s' failed!", PYAGENT_FUNC_NAMES[PYAGENT_FUNC_INIT]);
								status = CommandStatus::EXEC_FAIL;
							}
							else{
								// we do not expect any results
								Py_DECREF(result);
							}
						}
					}
				}
			}
		}
		#endif
		if(status == CommandStatus::SUCCESS){
			_trainingMode = true;
			initializeTraining();
			INFO_F("%s", getTrainingModeHint());
		}
	}
	else{
		INFO("Already in TRAINING MODE.");
	}
	return status;
}

CommandStatus Arena::cmdStopTraining(int argc, const char * argv[])
{
	if(_trainingMode){
		_trainingMode = false;
		INFO("TRAINING MODE deactivated.");
		#ifdef ARENA_USE_PYTHON
		if(_pyAgentUsed){
			// training stop callback
			INFO("Shutting down agent...");
			PyObject * f = _agentFuncs[PYAGENT_FUNC_STOP];
			if(f){
				PyObject * result = PyObject_CallObject(f, NULL);
				if(result == NULL){
					PyErr_Print();
					ERROR_F("Call to Function '%s' failed!", PYAGENT_FUNC_NAMES[PYAGENT_FUNC_STOP]);
				}
				else{
					// throw away result, whatever that is
					Py_DECREF(result);
				}
			}
			for(int i = 0; i < PYAGENT_FUNC_NUM; i++){
				Py_XDECREF(_agentFuncs[i]);
				_agentFuncs[i] = NULL;
			}
			Py_XDECREF(_agentModule);
			_agentModule = NULL;
			_pyAgentUsed = false;
			INFO("Done!");
		}
		#endif
	}
	else{
		INFO("Currently not in TRAINING MODE.");
	}
	return CommandStatus::SUCCESS;
}

CommandStatus Arena::cmdSetFPS(int argc, const char* argv[])
{
	if(argc != 2){
		return CommandStatus::INVALID_ARG;
	}

	int fps = atoi(argv[1]);
	if(fps < 1)
		fps = 1;
	if(!strcmp(argv[0], "physics")){
		_SETTINGS->physics.fps = fps;
	}
	else if(!strcmp(argv[0], "video")){
		_SETTINGS->video.fps = fps;
	}
	else{
		ERROR("First Parameter must be 'video' or 'physics'");
		return CommandStatus::INVALID_ARG;
	}

	return CommandStatus::SUCCESS;
	
}
