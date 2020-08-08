#include "arena.h"

// sigint handler
static Arena* ARENA =  NULL;// will be pointing to single arena instance after initialization
void sigintHandler(int state){
	ARENA->exitApplication();
}

Arena::Arena(): _console(NULL), _monospaceRegular(NULL), _monospaceBold(NULL), _level(NULL), _videoFPSView(NULL), _physicsFPSView(NULL),
				_consoleEnabled(false), _episodeEnd(0), _episodeCount(0),
				_physicsStepsRemainder(0), _agentMeasure(100), _agentPostMeasure(100), _simulationMeasure(100)
{
	#ifdef ARENA_USE_PYTHON
	for(int i = 0; i < PYAGENT_FUNC_NUM; i++){
		_agentFuncs[i] = NULL;
	}
	_agentModule = NULL;
	_pyAgentUsed = false;
	#endif
}

int Arena::init(int argc, char ** argv)
{
	_run = false;
	/* parse cmd options */
	bool disable_video = false;
	const char* log_file = NULL;
	_trainingMode = false;
	int command_index = -1;
	int arg_i = 1;
	while(arg_i < argc){
		if(!strcmp(argv[arg_i], "--disable-video")){// no video 
			disable_video = true;
		}
		else if(!strcmp(argv[arg_i], "--logfile")){// logfile
			if(arg_i < argc-1){
				log_file = argv[arg_i+1];
				arg_i++;// skip next argument (logfile path)
			}
			else{
				printf("No path given for logfile!\n");
				exit(1);
			}
		}
		else if((!strcmp(argv[arg_i], "--help") ||
						!strcmp(argv[arg_i], "-h"))){ // display help
			puts(getHelpString());
			exit(0);
		}
		else if(!strcmp(argv[arg_i], "--run")){
			if(arg_i+1 < argc){
				command_index = arg_i+1;
				arg_i++;// skip next argument run command
			}
			else{
				printf("No commands given for option --run!\n");
				exit(0);
			}
		}
		else{
			printf("Unknown commandline option '%s'\n", argv[arg_i]);
			exit(1);
		}
		arg_i++;
	}
	if(log_file)
		printf("Logging to '%s'.\n", log_file);

	if(_trainingMode)
		puts(getTrainingModeHint());

	/* create logger */
	Z_LOG->createLog(true, log_file);
	initCommands();

	/* init global settings*/
	std::string package_path = ".";
	std::string settings_file_path = package_path + "/settings.st";
	_SETTINGS_OBJ->init(settings_file_path.c_str());


	if(disable_video){
		_SETTINGS->video.enabled = false;
	}

	if(_SETTINGS->video.enabled)
		INFO("Starting Application in VIDEO MODE.");
	else
		INFO("Starting Application in COMMANDLINE MODE.");
	/* video setup */
	if(_SETTINGS->video.enabled) {

		/* initialize framework */
		SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");// we have our own sigint handler
		unsigned int flags = Z_RESIZABLE;
		if(_SETTINGS->video.vsync)
			flags |= Z_V_SYNC;
		if(_SETTINGS->video.fullscreen)
			flags |= Z_FULLSCREEN;
		if(_SETTINGS->video.maximized)
			flags |= Z_MAXIMIZED;
		if(Z_FW->init(	_SETTINGS->video.resolution_w,
						_SETTINGS->video.resolution_h,
						_SETTINGS->video.window_x,
						_SETTINGS->video.window_y,
						_SETTINGS->video.msaa,
						flags)){
			ERROR("Failed to initialize Framework!");
			return -1;
		}
		
		/* initialize opengl renderer */
		if(_RENDERER->init()){
			ERROR("Could not initialize Renderer!");
			return -1;
		}

		/* setting background color */
		glClearColor(1,1,1,1);

	
		INFO("Loading Fonts...");
		
		/* loading default font */
		_monospaceRegular = new zFont();
		_monospaceBold = new zFont();
		std::string p = package_path + "/data/fonts/Bitstream_Regular.ttf";
		INFO_F("-> loading %s", p.c_str());
		if(_monospaceRegular->loadFromFile(p.c_str())){
			return -1;
		}
		_monospaceRegular->renderMap(Z_NORMAL, _SETTINGS->gui.font_size);
		_monospaceBold = new zFont();
		p = package_path + "/data/fonts/Bitstream_Bold.ttf";
		INFO_F("-> loading %s", p.c_str());
		if(_monospaceBold->loadFromFile(p.c_str())){
			return -1;
		}
		_monospaceBold->renderMap(Z_NORMAL, _SETTINGS->gui.font_size);
		_monospaceBold->renderMap(Z_BIG, _SETTINGS->gui.font_size*1.5f);

		/* creating console */
		_console = new Console(_monospaceRegular, Z_NORMAL, "command_history.txt");
		_console->enable();

		/* creating fps counter */
		_videoFPSView = new zTextView(_monospaceRegular, Z_NORMAL, 32);
		_videoFPSView->setAlignment(-1, 0, 0, true);
		_videoFPSView->setColor(1,1,1,1);
		_physicsFPSView = new zTextView(_monospaceRegular, Z_NORMAL, 32);
		_physicsFPSView->setAlignment(-1, 0, 0, true);
		_physicsFPSView->setColor(1,1,1,1);
		_updateFPSView = new zTextView(_monospaceRegular, Z_NORMAL, 32);
		_updateFPSView->setAlignment(-1, 0, 0, true);
		_updateFPSView->setColor(1,1,1,1);
		_episodeCountView = new zTextView(_monospaceRegular, Z_NORMAL, 32);
		_episodeCountView->setAlignment(-1, 0, 0, true);
		_episodeCountView->setColor(1,1,1,1);
		_realTimeView = new zTextView(_monospaceRegular, Z_NORMAL, 32);
		_realTimeView->setAlignment(-1, 0, 0, true);
		_realTimeView->setColor(1,1,1,1);
		_rewardCountView = new zTextView(_monospaceRegular, Z_NORMAL, 32);
		_rewardCountView->setAlignment(-1, 0, 0, true);
		_rewardCountView->setColor(1,1,1,1);

		/* creating video disabled text */
		_videoDisabledText = new zTextView(_monospaceBold, Z_BIG, "Video mode disabled! Press 'F3' to enable.");
		_videoDisabledText->setAlignment(0, 0, 0, true);
		_videoDisabledText->setColor(1,1,1,1);

		/* initial resize call */
		_videoDisabled = false;

		/* initial camera settings */
		_translateCamera = false;
		_rotateCamera = false;
		_camera.set(zVector2D(_SETTINGS->gui.camera_x, _SETTINGS->gui.camera_y),
						_SETTINGS->gui.camera_zoom, f_rad(_SETTINGS->gui.camera_rotation));
		_camera.setZoomFactor(_SETTINGS->gui.camera_zoom_factor);
		_camera.refresh();
	}
	/* initialize physics */
	_PHYSICS->init();
	_PHYSICS->setIterations(_SETTINGS->physics.position_iterations,
							_SETTINGS->physics.velocity_iterations);

	/* register this object as contact listener */
	_PHYSICS_WORLD->SetContactListener((b2ContactListener*)this);


	/* initialize burger */
	_burger = new Burger(_PHYSICS_WORLD);

	/* creating environments */
	_numEnvs = _SETTINGS->training.num_envs;
	if(_numEnvs <= 0){
		_numEnvs = 1;
	}
	_envs = new Environment[_numEnvs];

	// find closest square
	_envsX = ceil(sqrt(_numEnvs));
	_envsY = _envsX;

	// create actions, rewards and _dones array
	_actions = new int[_numEnvs];
	_dones = new bool[_numEnvs];

	_numThreads = _SETTINGS->training.num_threads;
	if(_numThreads == 0){// at least 1 thread
		_numThreads = 1;
	}else if(_numThreads < 0){// automatically determine number of threads
		_numThreads = SDL_GetCPUCount();
	}
	if(_numThreads > _numEnvs){// more threads than environments does not make any sense
		_numThreads = _numEnvs;
	}
	INFO_F("Num Threads: %d", _numThreads);
	_threads = new EnvironmentThread[_numThreads];
	int envs_per_thread = _numEnvs/_numThreads;
	int envs_per_last_thread = envs_per_thread + _numEnvs%_numThreads;
	for(int i = 0; i < _numThreads; i++){
		_threads[i].init(_envs, (i==_numThreads-1) ? envs_per_last_thread : envs_per_thread, i*envs_per_thread, _actions);
	}

	// init key status
	memset(_keysPressed, 0, sizeof(_keysPressed));
	 
	// setup timer if video is disabled
	if(_SETTINGS->video.enabled == false){
		// init timer
		SDL_Init(SDL_INIT_TIMER);
	}

	// initialize sigint-handler
	ARENA = this;
	signal(SIGINT, &sigintHandler);

	/* initialize ros */
	#ifdef ARENA_USE_ROS
	if(_SETTINGS->ros.enabled){
		INFO("Initializing ROS...");
		if(initROS(argc, argv) < 0){
			ERROR("Failed to initialize ROS!");
			return -1;
		}
	}
	#else
	if(_SETTINGS->ros.enabled){
		WARNING("Application was not compiled with ARENA_USE_ROS, enabling ros takes no effect!");
	}
	#endif

	/* initialize python interpreter */
	#ifdef ARENA_USE_PYTHON
	INFO_F("Initializing python interpreter with program name '%s'", argv[0]);
	int len = strlen(argv[0]);
	#ifdef ARENA_PYTHON_VERSION_3
		_pyProgName = Py_DecodeLocale(argv[0], NULL);
	#else
		_pyProgName = new char[len+1];
		strcpy(_pyProgName, argv[0]);
	#endif
	Py_SetProgramName(_pyProgName);
	_pyArg0 = NULL;
	Py_InitializeEx(0);
	#endif

	INFO("Initialization done! Running Arena...\n");
	_run = true;

	/* load initial level */
	if(_SETTINGS->stage.initial_level >= 0){
		loadLevel(_SETTINGS->stage.initial_level, true);
	}
	else{
		/* init training if not already done through loadLevel*/
		initializeTraining();
	}

	if(command_index >= 0){
		INFO_F("Running initial command: > %s", argv[command_index]);
		command(argv[command_index]);
	}

	/* initial resize */
	if(_SETTINGS->video.enabled){
		resize();

		refreshEpisodeCounter();
		refreshRewardCounter();
		refreshFPSCounter();
	}

	return 0;
}

void Arena::exitApplication(){
	INFO("\nExiting application...");
	if(_run == false)// already tried to quit -> force
		exit(1);
	static const char * prompt_text = "TRAINING MODE active. Do you really want to exit the application?";
	if(_trainingMode && _SETTINGS->video.enabled){
		// creating message box and ask user whether he really wants to exit
		static const SDL_MessageBoxButtonData buttons[2] = {
				{0, 0, "NO, keep on training."},// type, button_id, text
				{0, 1, "YES, stop training!"}
		};
		static const SDL_MessageBoxColorScheme colors = {
			{
				{210, 210, 210},// message box background color
				{20, 20, 20},	// text color
				{20, 20, 20},	// button border
				{170, 170, 170},// button background
				{255, 255, 255} // button text selected
			}
		};
		const SDL_MessageBoxData data = {
			SDL_MESSAGEBOX_WARNING,
			Z_FW->getMainWindow(),
			"Exit?",
			prompt_text,
			2,
			buttons,
			&colors
		};
		int buttonid;
		if(SDL_ShowMessageBox(&data, &buttonid)){
			ERROR("Failed to show message box!");
		}
		else if(buttonid == 1){ /// user clicked yes
			_run = false;
		}
	}
	else{
		if(_trainingMode){
			printf("%s (y/n): ", prompt_text);
			std::string str;
			getline(std::cin, str);
			zStringTools::toLower(str);
			if(str == "yes" || str == "y"){
				_run = false;
			}
		}
		else{
			_run = false;
		}
	}
	if(_run){
		INFO("Exiting cancelled!");
	}
}

void Arena::quit()
{
	INFO("\nShutting down Application...");

	// shutting down environment threads
	delete[]_threads;

	delete[](_actions);
	delete[](_dones);

	/* shutting down python interpreter */

	#ifdef ARENA_USE_PYTHON
	if(_trainingMode)
		cmdStopTraining(0, NULL);
	Py_Finalize();
	#ifdef ARENA_PYTHON_VERSION_3
		PyMem_RawFree(_pyProgName);
		PyMem_RawFree(_pyArg0);
	#else
		delete[](_pyArg0);
		delete[]_pyProgName;
	#endif
	#endif

	/* free console */
	delete _console;
	_console = NULL;

	/* free current level */
	delete _level;
	_level = NULL;


	/* shutting down physics */
	_PHYSICS->del();

	if(_SETTINGS->video.enabled){
		/* free resources */
		delete _monospaceRegular;
		_monospaceRegular = NULL;

		delete _monospaceBold;
		_monospaceBold = NULL;

		/* free text views */
		delete _videoFPSView;
		_videoFPSView = NULL;

		delete _physicsFPSView;
		_physicsFPSView = NULL;

		delete _episodeCountView;
		_episodeCountView = NULL;

		delete _realTimeView;
		_realTimeView = NULL;

		delete _rewardCountView;
		_rewardCountView = NULL;

		/* shutting down renderer */
		_RENDERER->del();

		/* shutting down framework */
		Z_FW->del();
	}

	INFO("Done!");
	/* close logger */
	Z_LOG->del();
}

int Arena::loadLevel(int level, bool random)
{
	for(int i = 0; i < _numEnvs; i++)
	{
		if(_envs[i].loadLevel(level) < 0)
			return -1;
	}
	initializeTraining();
	return 0;
}

void Arena::reset(bool reset_burger){
	for(int i = 0; i < _numEnvs; i++)
	{
		_envs[i].reset(reset_burger);
	}

	// refresh reward counter
	if(_SETTINGS->video.enabled)
		refreshRewardCounter();
}

void Arena::initializeTraining(){
	_meanSuccess.reset();
	_meanReward.reset();
	_trainingStartTime = SDL_GetTicks();
	_episodeCount = 0;
	if(_SETTINGS->video.enabled)
		refreshEpisodeCounter();
	
}

void Arena::run(){
	_updateTimer.reset();
	_physicsTimer.reset();
	_videoTimer.reset();
	int iteration = 1;
	int next_video_update_it = 1;// iteration at which to perform the next video update
	int remainder = 0;
	if(_SETTINGS->video.enabled){/* video */
		while(_run){
			_updateTimer.setTargetFPS(_SETTINGS->physics.fps);
			_videoTimer.setTargetFPS(_SETTINGS->video.fps);
			_physicsTimer.setTargetFPS(_SETTINGS->physics.fps);

			if(iteration == next_video_update_it){
				zEventList evt;	
				if(Z_FW->update(&evt) == 0)/* user closed window */
					exitApplication();

				if(_consoleEnabled)
					updateConsole(evt);
				processEvents(evt);

				if(!_videoDisabled){
					Z_FW->clearScreen();
					/* render arena */
					render();

					/* render GUI (console, stats, etc)*/
					renderGUI();
					Z_FW->flipScreen();
				}

				iteration = 0;
				next_video_update_it = 1;
				int update_fps = f_round(_updateTimer.getCurrentFPS());
				if(_SETTINGS->video.fps < update_fps){
					int f = remainder+update_fps;
					next_video_update_it = f/_SETTINGS->video.fps;
					remainder = (f%_SETTINGS->video.fps);
				}
				// measure fps
				_videoTimer.update(false);
			}
			update();
			int d = _updateTimer.update(!(_trainingMode && _videoDisabled));// no fps limit if in training mode and video disabled
			iteration++;
		}
	}
	else{/* no video */
		while(_run){
			_updateTimer.setTargetFPS(_SETTINGS->physics.fps);
			_physicsTimer.setTargetFPS(_SETTINGS->physics.fps);
			update();
			_updateTimer.update(!_trainingMode);// no fps limit when in training mode
		}
	}
}


void Arena::renderVideoDisabledScreen()
{
	Z_FW->clearScreen();
	render();
	renderGUI();
	// render overlay
	int ms = glIsEnabled(GL_MULTISAMPLE);
	if(ms)
		glDisable(GL_MULTISAMPLE);

	// render background
	_RENDERER->useColorShader();
	_RENDERER->setScreenMatrix();
	_RENDERER->resetCameraMatrix();
	Z_SHADER->setColor(0.2f, 0.2f, 0.2f, 0.8f);
	zRect r;
	_RENDERER->getScreenRect(&r);
	_RENDERER->set2DTransform(zVector2D(r.x, r.y), zVector2D(r.w*2, r.h*2));
	_RENDERER->bindGO(GO_CENTERED_QUAD);
	_RENDERER->drawGO();

	// render text
	_RENDERER->useTextShader();
	_RENDERER->setScreenMatrix();
	_RENDERER->resetCameraMatrix();
	_videoDisabledText->render();

	if(ms)
		glEnable(GL_MULTISAMPLE);
	Z_FW->flipScreen();
}

void Arena::render()
{
	Quadrangle q;
	_RENDERER->getTransformedGLRect(_camera.getInverseMatrix(), &q);
	zRect r;
	q.getAABB(&r);
	int screen_w = Z_FW->getWindowW();
	int screen_h = Z_FW->getWindowH();
	int w_per_env = screen_w/_envsX;
	int h_per_env = screen_h/_envsY;
	int w_last_env = w_per_env + screen_w%_envsX;
	int h_last_env = h_per_env + screen_h%_envsY;
	for(int i = 0; i < _numEnvs; i++){
		int x_index = i%_envsX;
		int y_index = _envsY - 1 -(i/_envsY);
		int w =  x_index == (_envsX-1) ? w_last_env : w_per_env;
		int h =  y_index == (_envsY-1) ? h_last_env : h_per_env;
		int x = x_index*w_per_env;
		int y = y_index*h_per_env;
		_RENDERER->refreshProjectionMatrizes(w, h);
		glViewport(x, y, w, h);
		_envs[i].render(_camera, r);
	}
	glViewport(0, 0, screen_w, screen_h);
	_RENDERER->refreshProjectionMatrizes(screen_w, screen_h);
}

void Arena::renderGUI()
{
	// disable multisampling for gui
	int ms = glIsEnabled(GL_MULTISAMPLE);
	if(ms)
		glDisable(GL_MULTISAMPLE);

	/*** render stats ***/
	if(_SETTINGS->gui.show_stats){
		// render background
		_RENDERER->useColorShader();
		_RENDERER->setScreenMatrix();
		_RENDERER->resetCameraMatrix();
		Z_SHADER->setColor(0.2, 0.2, 0.2, 0.8);
		_RENDERER->drawRect(_statsBackgroundRect);

		// render text
		_RENDERER->useTextShader();
		_RENDERER->setScreenMatrix();
		_RENDERER->resetCameraMatrix();
		_videoFPSView->render();
		_updateFPSView->render();
		_physicsFPSView->render();
		_episodeCountView->render();
		_rewardCountView->render();
		_realTimeView->render();
	}
	
	/*** render console ***/
	if(_consoleEnabled){
		// render background
		_RENDERER->useColorShader();
		_RENDERER->setScreenMatrix();
		_RENDERER->resetCameraMatrix();
		_console->renderBox();

		// render text
		_RENDERER->useTextShader();
		_RENDERER->setScreenMatrix();
		_RENDERER->resetCameraMatrix();
		_console->renderText();
	}
	/************************/

	// restore multisampling
	if(ms)
		glEnable(GL_MULTISAMPLE);
}

void Arena::updateConsole(zEventList & evt){
	if(_console->update(&evt)){
		const char * cmd = _console->getCommand();
		INFO_F("> %s", cmd);
		CommandStatus s = command(cmd);
		switch(s)
		{
		case CommandStatus::UNKNOWN_COMMAND:{
			ERROR_F("Unknown command '%s'!", cmd);
		}break;
		case CommandStatus::EXEC_FAIL:{
			ERROR_F("Failed to Execute command '%s'!", cmd);
		}break;
		case CommandStatus::INVALID_ARG:{
			ERROR_F("Invalid arguments in command '%s'!", cmd);
		}break;
		case CommandStatus::SUCCESS:{
			INFO(" ");
		}break;
		}
	}
}

int Arena::update()
{
	// update key control
	_actions[0] = -1;
	int action = -1;
	if(!_trainingMode && _episodeEnd == 0 && !_videoDisabled){
		if(_keysPressed[UP]){
			action = Burger::FORWARD;
			if(_keysPressed[LEFT]){
				action = Burger::FORWARD_LEFT;
			}
			else if(_keysPressed[RIGHT]){
				action = Burger::FORWARD_RIGHT;
			}
		}else if(_keysPressed[DOWN]){
			action = Burger::BACKWARD;
		}
		else{
			if(_keysPressed[LEFT]){
				action = Burger::FORWARD_STRONG_LEFT;
			}
			else if(_keysPressed[RIGHT]){
				action = Burger::FORWARD_STRONG_RIGHT;
			}
		}
	}


	#ifdef ARENA_USE_PYTHON
	if(_trainingMode && _pyAgentUsed){
		_agentMeasure.reset();
		_agentReward = 0.f;
		if(_agentFuncs[PYAGENT_FUNC_PRE_STEP] != NULL){
			// creating list
			PyObject * args = PyTuple_New(1);
			PyTuple_SetItem(args, 0, packAllPyObservation());
			PyObject * result = PyObject_CallObject(_agentFuncs[PYAGENT_FUNC_PRE_STEP], args);
			Py_DECREF(args);
			if(result == NULL){
				PyErr_Print();
				ERROR_F("Call to function '%s' in Python agent failed", PYAGENT_FUNC_NAMES[PYAGENT_FUNC_PRE_STEP]);
				cmdStopTraining(0, NULL);
			}
			else{
				if(PyList_Check(result)){// get multiple actions
					if(PyList_Size(result) != _numEnvs){// check number of elements
						ERROR_F("Call to function '%s' in Python agent failed: Size of returned list (%d) does not match number of environments (%d)!", PYAGENT_FUNC_NAMES[PYAGENT_FUNC_PRE_STEP], PyList_Size(result), _numEnvs);
						cmdStopTraining(0, NULL);
					}
					else{
						for(int i = 0; i < _numEnvs; i++){
							_actions[i] = PyLong_AsLong(PyList_GetItem(result, i));
						}
					}
				}
				else if(PyLong_Check(result)){// get single action
					action = (int)PyLong_AsLong(result); 
				}
				else{
					ERROR_F("Call to function '%s' in Python agent failed: Expected List or Long for return value!", PYAGENT_FUNC_NAMES[PYAGENT_FUNC_PRE_STEP]);
					cmdStopTraining(0, NULL);
				}
				Py_DECREF(result);
			}
		}
		_agentMeasure.update(false);
	}
	#endif

	if(action != -1){
		for(int i = 0; i < _numEnvs; i++){
			_actions[i] = action;
		}
	}

	_physicsTimer.checkLastUpdate();

	if(_actions[0] >= 0){
		// threaded simulation step
		_simulationMeasure.reset();
		for(int i = 0; i < _numThreads; i++){
			_threads[i].step();
		}

		for(int i = 0; i < _numThreads; i++){
			_threads[i].wait_finish();
		}
		_simulationMeasure.update(false);

		// check whether episodes ended and reset environment
		bool episode_over = false;
		for(int i = 0; i < _numEnvs; i++){
			Environment::EpisodeState s = _envs[i].getEpisodeState();
			if(s != Environment::RUNNING){
				// adding success value to success buffer
				_meanSuccess.push((s == Environment::POSITIVE_END) ? 1 : 0);
				_meanSuccess.calculateMean();
				_dones[i] = true;

				// adding reward to total reward buffer
				_meanReward.push(_envs[i].getTotalReward());
				_meanReward.calculateMean();

				_episodeCount++;

				// show results
				printEpisodeResults(_envs[i].getTotalReward());

				// reset environment
				_envs[i].reset(s == Environment::NEGATIVE_END);
				episode_over = true;
			}
			else{
				_dones[i] = false;
			}
		}
		if(episode_over){
			if(_SETTINGS->video.enabled){
				refreshEpisodeCounter();
				refreshRewardCounter();
			}
		}

		// measuring FPS
		_physicsTimer.update(false);

		// call agents post step function
		#ifdef ARENA_USE_PYTHON
		if(_trainingMode && _pyAgentUsed){
			_agentReward += _reward;
			_agentPostMeasure.reset();
			if(_agentFuncs[PYAGENT_FUNC_POST_STEP] != NULL){
				PyObject * args = PyTuple_New(6);
				PyTuple_SetItem(args, 0, packAllPyObservation());
				PyTuple_SetItem(args, 1, packAllPyActions());
				PyTuple_SetItem(args, 2, packAllPyRewards());
				PyTuple_SetItem(args, 3, PyFloat_FromDouble(_meanReward.getMean()));
				PyTuple_SetItem(args, 4, packAllPyDones());
				PyTuple_SetItem(args, 5, PyFloat_FromDouble(_meanSuccess.getMean()));
				PyObject * result = PyObject_CallObject(_agentFuncs[PYAGENT_FUNC_POST_STEP], args);
				Py_DECREF(args);
				if(result == NULL){
					PyErr_Print();
					ERROR_F("Call to function '%s' in Python agent failed", PYAGENT_FUNC_NAMES[PYAGENT_FUNC_POST_STEP]);
					cmdStopTraining(0, NULL);
				}
				else{
					int done = PyLong_AsLong(result); 
					Py_DECREF(result);
					if(done){
						cmdStopTraining(0, NULL);
						INFO("Training done!");
					}
				}
			}
			_agentPostMeasure.update(false);
			_agentReward = 0.f;
		}
		#endif
	}

	// setting camera to current burger position if enabled
	if(	_SETTINGS->video.enabled &&
		_SETTINGS->gui.camera_follow &&
		!_translateCamera){
		b2Transform t = _burger->getBody()->GetTransform();
		if(_SETTINGS->gui.camera_follow == 2){
			_camera.setRotation(t.q.GetAngle()-M_PI/2.0f);
		}
		_camera.setPos(zVector2D(t.p.x, t.p.y));
		_camera.refresh();
	}


	return 0;
}

void Arena::screenshot(const char * path)
{
	int windowW = Z_FW->getWindowW();
	int windowH = Z_FW->getWindowH();
	SDL_Surface *s = SDL_CreateRGBSurface(0, windowW, windowH, 32, 0x000000FF,0x0000FF00,0x00FF0000,0xFF000000);
	glReadPixels(0, 0, windowW, windowH, GL_RGBA, GL_UNSIGNED_BYTE, s->pixels);
	int size = windowW*windowH;
	Uint32 * pixels = ((Uint32*)s->pixels);
	for(int i =0; i < size/2; i++){
		int x = i%windowW;
		int y = i/windowW;
		int mirrored_index = (windowH-y-1)*windowW + x;
		Uint32 p = pixels[mirrored_index];
		pixels[mirrored_index] = pixels[i];
		pixels[i] = p;
	}
	if(SDL_SaveBMP(s, path)){
		puts(SDL_GetError());
	}
	SDL_FreeSurface(s);
}

void Arena::printEpisodeResults(float total_reward)
{
		char time_buffer[32];
		Timer::getTimeString(SDL_GetTicks()-_trainingStartTime, time_buffer, 32);
		INFO_F(	"\nEpisode %d over:\n"
				"  Reward:          %.3f\n"
				"  Mean reward:     %.3f (last 100 episodes)\n"
				"  Speed:           %.1f steps/sec (x%d)\n"
				"  Agent Time:      %.3fms = (%.3f + %.3f)ms\n"
				"  Simulation Time: %.3fms\n"
				"  Time elapsed:    %s\n"
				"  Success Rate:    %.0f%%",
			_episodeCount,
			total_reward,
			_meanReward.getMean(),
			_physicsTimer.getCurrentFPS(), _numEnvs,
			_agentMeasure.getCurrentFrameTime() + _agentPostMeasure.getCurrentFrameTime(),
			_agentMeasure.getCurrentFrameTime(), _agentPostMeasure.getCurrentFrameTime(),
			_simulationMeasure.getCurrentFrameTime(),
			time_buffer, _meanSuccess.getMean()*100);
}


bool Arena::getGoalDistance(float & l2, float & angle)
{
	if(_level != NULL && _level->getGoal() != NULL){
		b2Vec2 goal_pos = _level->getGoalPosition();
		goal_pos = _burger->getBody()->GetLocalPoint(goal_pos);
		l2 = goal_pos.Length();
		angle = f_deg(zVector2D::signedAngle(zVector2D(goal_pos.x, goal_pos.y), zVector2D(1, 0)));
		return true;
	}
	return false;
}


int Arena::processEvents(zEventList & evtList)
{
	for(zEventList::iterator it = evtList.begin(); it != evtList.end(); it++)
	{
		switch(it->type)
		{
		/* window resize */
		case SDL_WINDOWEVENT: {
			if(it->window.event == SDL_WINDOWEVENT_RESIZED){
				_SETTINGS->video.resolution_w = it->window.data1;
				_SETTINGS->video.resolution_h = it->window.data2;
				resize();
			}
			else if(it->window.event == SDL_WINDOWEVENT_MOVED){
				_SETTINGS->video.window_x = it->window.data1;
				_SETTINGS->video.window_y = it->window.data2;
				_SETTINGS->video.maximized = 0;
			}
			else if(it->window.event == SDL_WINDOWEVENT_MAXIMIZED){
				_SETTINGS->video.maximized = 1;
				_SETTINGS->video.window_x = it->window.data1;
				_SETTINGS->video.window_y = it->window.data2;
			}
			else if(it->window.event == SDL_WINDOWEVENT_RESTORED){
				_SETTINGS->video.maximized = 0;
			}
			else if(it->window.event == SDL_WINDOWEVENT_EXPOSED){
				if(_videoDisabled) {// redraw screen
					renderVideoDisabledScreen();
				}
			}
		}break;
		case SDL_USEREVENT:{
			if(it->user.code == Z_EVENT_TIMER_HALF){
				refreshFPSCounter();
			}
		}break;
		case SDL_MOUSEMOTION:{
			zVector2D t(it->motion.xrel, it->motion.yrel);
			_RENDERER->toGLCoordinates(t, true);
			if(_translateCamera){
				_camera.translateScaled(-t);
				_camera.refresh();

				//updating settings
				_SETTINGS->gui.camera_x = _camera.getPos().x;
				_SETTINGS->gui.camera_y = _camera.getPos().y;
			}
			if(_rotateCamera){
				_camera.rotate(t.x);
				_camera.refresh();

				//updating settings
				_SETTINGS->gui.camera_rotation = f_deg(_camera.getRotation());
			}
		}break;
		case SDL_MOUSEBUTTONUP:
		case SDL_MOUSEBUTTONDOWN:{
			if(it->button.button == SDL_BUTTON_RIGHT){
				_translateCamera = (it->button.type == SDL_MOUSEBUTTONDOWN);
			}
			else if(it->button.button == SDL_BUTTON_MIDDLE){
				_rotateCamera = (it->button.type == SDL_MOUSEBUTTONDOWN);
			}
		}break;
		case SDL_MOUSEWHEEL:{
			_camera.zoomExp(it->wheel.y);
			_camera.refresh();

			// updating settings
			_SETTINGS->gui.camera_zoom = _camera.getZoom();
		}break;
		case SDL_KEYUP:
		case SDL_KEYDOWN:{
			SDL_Keycode sym = it->key.keysym.sym;
			Uint16 mod = it->key.keysym.mod;
			bool down = (it->key.type == SDL_KEYDOWN);
			bool event = (down && it->key.repeat == 0);
			if(sym == SDLK_F1)//toggle console
			{
				if(event){
					_consoleEnabled = !_consoleEnabled;
				}
			}
			else if(sym == SDLK_F3)//toggle video
			{
				if(event){
					_videoDisabled = !_videoDisabled;
					if(_videoDisabled){// video is now disabled -> render overlay
						renderVideoDisabledScreen();
					}
				}
			}
			else if(sym == _SETTINGS->keys.up){
				if(_trainingMode){
					if(event)
						showTrainingModeWarning();
				}
				else
					_keysPressed[UP] = down;
			}
			else if(sym == _SETTINGS->keys.left){
				if(_trainingMode){
					if(event)
						showTrainingModeWarning();
				}
				else
					_keysPressed[LEFT] = down;
			}
			else if(sym == _SETTINGS->keys.down){
				if(_trainingMode){
					if(event)
						showTrainingModeWarning();
				}
				else
					_keysPressed[DOWN] = down;
			}
			else if(sym == _SETTINGS->keys.right){
				if(_trainingMode){
					if(event)
						showTrainingModeWarning();
				}
				else
					_keysPressed[RIGHT] = down;
			}
			else if(sym == _SETTINGS->keys.reset){
				if(event){
					if(_trainingMode){
						showTrainingModeWarning();
					}
					else{
						reset(_SETTINGS->stage.enable_position_reset);
					}
				}
			}
			else if(event && sym == SDLK_s && (mod & KMOD_CTRL)){
				command("save_settings");
			}
			else if(event && sym == SDLK_l && (mod & KMOD_CTRL)){
				command("load_settings");
			}
			else if(sym == SDLK_f){
				if(event){
					INFO_F("FPS: %.1f", Z_FW->getCurrentFPS());
				}
			}
		}break;
		}
	}

	return 0;
}

void Arena::refreshFPSCounter()
{
	static char buffer[128];
	sprintf(buffer, "FPS:         %.0f/%d", _videoTimer.getCurrentFPS(), _SETTINGS->video.fps);
	_videoFPSView->setText(buffer);

	sprintf(buffer, "Update FPS:  %.0f/%d", _updateTimer.getCurrentFPS(), _SETTINGS->physics.fps);
	_updateFPSView->setText(buffer);

	sprintf(buffer, "Steps*s:     %.0f*%.3f", _physicsTimer.getCurrentFPS(), _SETTINGS->physics.time_step);
	_physicsFPSView->setText(buffer);

	sprintf(buffer, "Real Time:   %.2f", _SETTINGS->physics.time_step*_physicsTimer.getCurrentFPS());
	_realTimeView->setText(buffer);
}

void Arena::refreshEpisodeCounter(){
	static char buffer[128];
	sprintf(buffer, "Episodes:    %d", _episodeCount);
	_episodeCountView->setText(buffer);
}

void Arena::refreshRewardCounter(){
	static char buffer[128];
	sprintf(buffer, "Mean Reward: %.1f", _meanReward.getMean());
	_rewardCountView->setText(buffer);
}

void Arena::resize(){
	/* update projection matrix */
	int windowW = Z_FW->getWindowW();
	int windowH = Z_FW->getWindowH();
	_RENDERER->refreshProjectionMatrizes(windowW, windowH);
	zRect r(0,0,0,0);
	float font_size = _videoFPSView->getFont()->getGlyphMap(_videoFPSView->getSize())->getSize();
	float vertical_offset = font_size*1.2f;
	float margin = 6;
	float text_w = font_size*13.2;
	int num_stats = 6;
	r.x = windowW-text_w-margin;
	r.y = font_size*0.6+margin;
	_videoFPSView->setAlignBox(r);
	_videoFPSView->align();
	r.y += vertical_offset;
	_updateFPSView->setAlignBox(r);
	_updateFPSView->align();
	r.y += vertical_offset;
	_physicsFPSView->setAlignBox(r);
	_physicsFPSView->align();
	r.y += vertical_offset;
	_realTimeView->setAlignBox(r);
	_realTimeView->align();
	r.y += vertical_offset;
	_rewardCountView->setAlignBox(r);
	_rewardCountView->align();
	r.y += vertical_offset;
	_episodeCountView->setAlignBox(r);
	_episodeCountView->align();
	_statsBackgroundRect.w = (text_w+2*margin)/2.f;
	_statsBackgroundRect.h = (num_stats*vertical_offset+2*margin)/2.f;
	_statsBackgroundRect.x = windowW-_statsBackgroundRect.w;
	_statsBackgroundRect.y = _statsBackgroundRect.h;

	_RENDERER->getScreenRect(&r);
	_videoDisabledText->setAlignBox(r);
	_videoDisabledText->align();

	// redraw screen if in video disabled mode
	if(_videoDisabled) {
		renderVideoDisabledScreen();
	}
}

