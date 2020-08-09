#ifndef ARENA_H
#define ARENA_H

// python interpreter (if enabled)
#ifdef ARENA_USE_PYTHON
	#include <Python.h>
	#define PYAGENT_FUNCTION_NUM 4
	enum PyAgentFunction{PYAGENT_FUNC_INIT, PYAGENT_FUNC_PRE_STEP, PYAGENT_FUNC_POST_STEP, PYAGENT_FUNC_STOP, PYAGENT_FUNC_NUM};
	extern const char * PYAGENT_FUNC_NAMES[PYAGENT_FUNC_NUM];
#endif // ARENA_USE_PYTHON
#include <iostream>
#include <engine/zFramework.h>
#include <engine/globalSettings.h>
#include <engine/camera.h>
#include <engine/timer.h>
#include <signal.h>
#include "Command.h"
#include "CommandRegister.h"
#include "applicationMode.h"
#include "Console.h"
#include "physicsWorld.h"
#include "level/level.h"
/* including levels */
#include "level/levelEmpty.h"
#include "level/levelStatic.h"
#include "level/levelDynamic.h"
#include "level/levelIndustry.h"
#include "level/levelSVG.h"

#include "burger.h"
#include "meanBuffer.h"
#include "environment.h"
#include <memory>
#ifdef ARENA_USE_ROS
#define ROS_NODE_NAME "arena"
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <ros/package.h>
// services for discrete actions.
#include <arena2d/InteractionDiscActs.h>
#include <arena2d/arenaCommand.h>
#endif
#include <pthread.h>
#include <chrono>
#include <thread>
class Arena
{
public:
	Arena();
	~Arena(){}
	int init(int argc, char ** argv);
	void run();
	void quit();

	/* command line tool */
	CommandStatus command(const char * c);
	CommandStatus command(const char * name, int argc, const char * argv[]);/* execute command */
	const CommandDescription* getCommand(const char * name); /* get command description */
	void getAllCommands(list<const CommandDescription*> & cmds); /* get all command descriptions */
	CommandStatus cmdHelp(int argc, const char * argv[]);
	CommandStatus cmdExit(int argc, const char * argv[]);
	CommandStatus cmdShow(int argc, const char * argv[]);
	CommandStatus cmdHide(int argc, const char * argv[]);
	CommandStatus cmdHideShow(int num_objects, const char * objects[], bool show);
	CommandStatus cmdLoadLevel(int argc, const char* argv[]);
	int loadLevel(int level, bool random);
	CommandStatus cmdReset(int argc, const char * argv[]);
	CommandStatus cmdSaveSettings(int argc, const char * argv[]);
	CommandStatus cmdLoadSettings(int argc, const char * argv[]);
	CommandStatus cmdSet(int argc, const char * argv[]);// set option in settings
	CommandStatus cmdStartTraining(int argc, const char * argv[]);
	CommandStatus cmdStopTraining(int argc, const char * argv[]);
	CommandStatus cmdSetFPS(int argc, const char * argv[]);
	CommandStatus cmdRealtime(int argc, const char * argv[]);

	/* reset current level */
	void reset(bool reset_burger=false);

	#ifdef ARENA_USE_PYTHON
	// pack observation of given environment into new PyObject
	PyObject* packPyObservation(int env_index);
	PyObject* packAllPyObservation();
	PyObject* packAllPyActions();
	PyObject* packAllPyRewards();
	PyObject* packAllPyDones();
	#endif
	#ifdef ARENA_USE_ROS
	/*
	run arena in a interactive way. arena will privide the its observation and others for the agent using ros service.
	*/
	void ros_run();
	#endif




	static const char* getHelpString(){
		return 	"options:\n"
				"--help -h                      display this info\n"
				"--disable-video                commandline only\n"
				"--logfile <filename>           log output to file\n"
				"--run <commands>               run simulator commands on startup (separated by ;)\n";
	}

	// exit application safely
	void exitApplication();

	// get relative goal distance (l2 distance and angle in rad)
	// return: true if distance could be retrieved (goal exists), else false
	bool getGoalDistance(float & l2, float & angle);

	void screenshot(const char * path);
private:
	/* private methods */
	void initCommands();
	void render();
	void renderGUI();
	void renderVideoDisabledScreen();
	void updateConsole(zEventList & evtList);
	int update();
	int processEvents(zEventList & evtList);
	void resize();/* resize event */
	void refreshFPSCounter();
	void refreshEpisodeCounter();
	void refreshRewardCounter();
	void physicsStep(Burger::Action a);
	void calculateMeanReward();
	void initializeTraining();
	#ifdef ARENA_USE_ROS
		int initROS(int argc,char* argv[]);
		bool remoteCommand(	arena2d::arenaCommand::Request & req,
						   	arena2d::arenaCommand::Response & res);
		bool interactionCallback( arena2d::InteractionDiscActs::Request & req, arena2d::InteractionDiscActs::Response & res);
		void step_update(); // only handle key-stroke, other logics such as prestep, poststep implemented in update will be move to interactionCallback
	#endif



	static void showTrainingModeWarning(){
		WARNING("TRAINING MODE active. Action might interfere with training process and thus will not be performed!");
	}
	static const char * getTrainingModeHint(){ 
		return "TRAINING MODE. Critical actions performed by user are blocked!";
	}
	void printEpisodeResults(float total_reward);
	zFont * _monospaceRegular;
	zFont * _monospaceBold;
	Console *_console;
	CommandRegister<Arena> _commands;
	zTextView * _videoFPSView;
	zTextView * _physicsFPSView;
	zTextView * _updateFPSView;
	zTextView * _episodeCountView;
	zTextView * _rewardCountView;
	zTextView * _realTimeView;
	zRect _statsBackgroundRect;
	volatile bool _run;

	Level * _level;
	Burger * _burger;

	Environment * _envs; // array of environments
	int _numEnvs;
	int _envsX; // number of environments displayed accross window width
	int _envsY; // number of environments displayed accross window height
	int _numThreads;
	EnvironmentThread * _threads;

	float _reward;
	float _agentReward; // accumulating reward over k successive frames
	float _totalReward; // accumulated reward for one episode
	MeanBuffer _meanReward;
	MeanBuffer _meanSuccess;
	MeanBuffer _envStepTime;
	int _episodeCount;
	int _episodeStepCount;
	int _physicsStepsRemainder;
	Timer _updateTimer;
	Timer _videoTimer;
	Timer _physicsTimer;
	Timer _simulationMeasure; // measuring time, the simulation takes
	Timer _agentPostMeasure;
	Timer _agentMeasure; // measuring time, the agent needs to perform a step
	float _timePerEpisode;
	int _episodeEnd;// -1: failure (time or wall hit), 0: not ended yet, 1: success (goal reached)
	int _lastAction;// last action performed by agent
	int * _actions;// action to perform in multiple environments
	bool * _dones;// episode done
	bool _consoleEnabled;
	bool _translateCamera;
	bool _rotateCamera;
	Camera _camera;
	bool _keysPressed[4];// keys for controlling the robot
	bool _trainingMode;// training mode active? (restricted user input)
	bool _videoDisabled;// temporarily disable rendering to increase performance of training
	zTextView * _videoDisabledText;
	Uint32 _trainingStartTime;

	#ifdef ARENA_USE_ROS
	bool _remoteStep;// is set to true if a call to serviceStep() occured during ros::spin()
	ros::Publisher _laserPub;		// publishing laser data
	ros::Publisher _distancePub; 	// publishing distance to goal
	ros::ServiceServer _commandService; // issuing commands remotely
	ros::ServiceServer interaction_server_;	// as the host the robot should perform action in the sim or real world and return observations to the planner.
	pthread_mutex_t interaction_mutex_; // the server for the interaction with a agent is kept running in a thread. we use this mutex to synchronize the speed of interaction-rate and update-rate, if FPS of _updateTimer was set.
	ros::CallbackQueue interaction_callback_queue_;
	std::unique_ptr<ros::AsyncSpinner> interaction_async_spinner_ptr_;
	#endif



	#ifdef ARENA_USE_PYTHON
	// embedded python callback functions
	bool _pyAgentUsed;// set to true if an embedded python agent was used during training
	#ifdef ARENA_PYTHON_VERSION_3
		wchar_t * _pyProgName;
		wchar_t * _pyArg0;
	#else// old python <= 2.7
		char * _pyProgName;
		char * _pyArg0;
	#endif
	PyObject * _agentFuncs[PYAGENT_FUNC_NUM];
	PyObject * _agentModule;
	#endif
};


#endif
