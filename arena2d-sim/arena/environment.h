#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <engine/zFramework.h>
#include <engine/globalSettings.h>
#include <engine/camera.h>
#include "level/level.h"
#include "burger.h"
#include "meanBuffer.h"
#include "level/levelEmpty.h"
#include "level/levelStatic.h"
#include "level/levelDynamic.h"
#include "level/levelIndustry.h"
#include "level/levelSVG.h"

// forward declaration for Environment
class Environment;

// EnvironmentThread representing thread that performs steps on multiple Environments
struct EnvironmentThread{
	EnvironmentThread();
	~EnvironmentThread();
	static int thread_func(void * data);
	static int thread_func_(void * data);
	void init(Environment * _env, int _num_envs, int _env_index, const volatile int * _action);
	void step();
	void wait_finish();
	Environment * env;// multiple envs
	const volatile int * action;// multple actions
	int num_envs;
	int env_index;// index of first environment
	enum State {WAITING, RUNNING, EXIT} state;
	SDL_mutex * state_mutex;
	SDL_cond * state_cond;
	SDL_Thread * thread;
};

class Environment : public b2ContactListener
{
public:
	enum EpisodeState{RUNNING, POSITIVE_END, NEGATIVE_END};
	Environment();
	~Environment();

	// load level returns 0 on success
	int loadLevel(int level);

	// perform simulation step
	// this function can be run for multiple environments simultaneously 
	void pre_step(int action);
	void stepAll(int action);// do pre_step, post_step and all step iterations
	void step();// do single step iteration
	void post_step();

	// Contact Listener
	void BeginContact(b2Contact * contact) override;
	void EndContact(b2Contact * contact) override;

	// reset environment, has to be called externally
	void reset(bool reset_burger);

	void refreshSettings();

	// returns true if episode has ended
	EpisodeState getEpisodeState(){return _episodeState;}

	b2World* getWorld(){return _world;}

	// render environment
	// viewport, camera-/projection-matrix must be set
	void render(const Camera & cam, const zRect & aabb);

	// get distance to goal
	void getGoalDistance(float & l2, float & angle);

	// get scan observation
	const float* getScan(int & num_samples){return _burger->getSamples(num_samples);}

	// get current reward
	float getReward(){return _reward;}

	// get total reward so far
	float getTotalReward(){return _totalReward;}

	void initializeTraining();

private:
	b2World * _world;
	f_physicsSettings _physicsSettings;
	f_trainingSettings _trainingSettings;// copy of settings to avoid read conflicts
	Level * _level;
	Burger * _burger;
	int _action;// action to perform in step
	float _distance;// distance to goal before step
	float _angle;// angle to goal before step
	float _reward;		// reward gained from current step
	float _totalReward; // accumulated reward for one episode
	float _episodeTime; // duration of current episode in seconds
	int _episodeStepCount; // number of steps in current episode
	int _episodeCount; // number of episodes since training start
	EpisodeState _episodeState; // episode has ended?
};

#endif

