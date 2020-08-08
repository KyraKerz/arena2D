#include "environment.h"

// EnvironmentThread
EnvironmentThread::EnvironmentThread(){
	state = WAITING;
	state_mutex = SDL_CreateMutex();
	state_cond = SDL_CreateCond();
	thread = SDL_CreateThread(EnvironmentThread::thread_func, "", (void*)this);
}

EnvironmentThread::~EnvironmentThread(){
	// terminate thread
	SDL_LockMutex(state_mutex);
	state = EXIT;
	SDL_UnlockMutex(state_mutex);
	SDL_CondSignal(state_cond);
	SDL_WaitThread(thread, NULL);

	SDL_DestroyMutex(state_mutex);
	SDL_DestroyCond(state_cond);
}

void EnvironmentThread::init(Environment * _env, int _num_envs, int _env_index, const int * _action){
	env = _env;
	num_envs = _num_envs;
	env_index = _env_index;
	action = _action;
}

void EnvironmentThread::step(){
	SDL_LockMutex(state_mutex);
	state = RUNNING;
	SDL_UnlockMutex(state_mutex);
	SDL_CondSignal(state_cond);
}

void EnvironmentThread::wait_finish(){
	SDL_LockMutex(state_mutex);
	while(state == RUNNING){
		SDL_CondWait(state_cond, state_mutex);
	}
	SDL_UnlockMutex(state_mutex);
}

int EnvironmentThread::thread_func(void * d){
	EnvironmentThread * data = (EnvironmentThread*)d;
	while(1){
		SDL_LockMutex(data->state_mutex);
		// wait for signal from main thread
		while(data->state == EnvironmentThread::WAITING){
			SDL_CondWait(data->state_cond, data->state_mutex);
		}

		// exit thread if state has been set to EXIT
		if(data->state == EnvironmentThread::EXIT){
			SDL_UnlockMutex(data->state_mutex);
			break;
		}

		// calculate steps
		for(int i = data->env_index; i < data->env_index+data->num_envs; i++){
			data->env[i].stepAll(data->action[i]);
		}

		// send signal done
		data->state = EnvironmentThread::WAITING;
		SDL_UnlockMutex(data->state_mutex);
		SDL_CondSignal(data->state_cond);
	}
	return 0;
}
/////// EnvironmentThread


Environment::Environment()
{
	_world = new b2World(b2Vec2(0,0));
	_world->SetContactListener((b2ContactListener*)this);
	_level = NULL;
	_burger = new Burger(_world);

	refreshSettings();
}

void Environment::refreshSettings()
{
	// copy training settings to avoid read conflicts on simultanious reads
	_physicsSettings = _SETTINGS->physics;
	_trainingSettings = _SETTINGS->training;
}

Environment::~Environment()
{
	delete _burger;
	delete _level;
	delete _world;
}

int Environment::loadLevel(int level)
{
	// free old level 
	delete _level;
	_level = NULL;

	switch(level){
	case Level::EMPTY_NO_BORDER: // empty level with no border
	case Level::EMPTY_BORDER:// empty level with borders
	{
		_level = new LevelEmpty(_world, level == Level::EMPTY_BORDER);
	}break;
	case Level::STATIC:
	{
		_level = new LevelStatic(_world, false);
	}break;
	case Level::DYNAMIC:
	{
		_level = new LevelDynamic(_world);
	}break;
	case Level::STATIC_DYNAMIC:
	{
		_level = new LevelStatic(_world, true);
	}break;
	case Level::SVG:
	{
		_level = new LevelSVG(_world);
	}break;
	default:
	{
		ERROR_F("Level %d is not implemented!", level);
		return -1;
	}
	}

	initializeTraining();
	if(_level != NULL){
		INFO_F("Level %d loaded!", level);
	}
	return 0;
}

void Environment::initializeTraining()
{
	reset(true);
	_totalReward = 0;
	_episodeCount = 0;
}

void Environment::pre_step(int action)
{
	_reward = 0.0f;
	_action = action;
	getGoalDistance(_distance, _angle);
}

void Environment::step()
{
	if(_episodeState != RUNNING || _action < 0 || _action >= Burger::NUM_ACTIONS)// episode over, has to be reset first
		return;
	_burger->performAction((Burger::Action)_action);
	_world->Step(_physicsSettings.time_step, _physicsSettings.velocity_iterations, _physicsSettings.position_iterations);
	_episodeTime += _physicsSettings.time_step;
	// time's up
	if(	_episodeTime > _trainingSettings.max_time &&
		_trainingSettings.max_time > 0.f && _level != NULL){
		_reward += _SETTINGS->training.reward_time_out;
		_episodeState = NEGATIVE_END;
	}
}

void Environment::stepAll(int action)
{
	pre_step(action);
	for(int i = 0; i < _physicsSettings.step_iterations; i++){
		step();
	}
	post_step();
}

void Environment::post_step()
{
	_episodeStepCount++;
	float distance_after, angle_after;
	getGoalDistance(distance_after, angle_after);

	// checking reward for distance to goal decreased/increased
	if(distance_after < _distance){
		_reward += _trainingSettings.reward_distance_to_goal_decreased;
	}
	else{
		_reward += _trainingSettings.reward_distance_to_goal_increased;
	}

	// updating level logic
	if(_level != NULL){
		const b2Transform & t = _burger->getBody()->GetTransform();
		_level->update(t);
		_reward += _level->getReward(t);
	}

	// update laser scan
	_burger->scan();


	_totalReward += _reward;
}

void Environment::getGoalDistance(float & l2, float & angle)
{
	if(_level != NULL && _level->getGoal() != NULL){
		b2Vec2 goal_pos = _level->getGoalPosition();
		goal_pos = _burger->getBody()->GetLocalPoint(goal_pos);
		l2 = goal_pos.Length();
		angle = f_deg(zVector2D::signedAngle(zVector2D(goal_pos.x, goal_pos.y), zVector2D(1, 0)));
	}
}

void Environment::reset(bool reset_burger)
{
	b2Vec2 burger_pos = _burger->getPosition();
	b2Vec2 burger_pos_before = burger_pos;
	if(reset_burger)
		burger_pos.Set(0,0);
	if(_level != NULL)// reset level
		_level->reset(burger_pos, reset_burger);
	
	// reset burger
	if(burger_pos != burger_pos_before)
		_burger->reset(burger_pos);

	// reset trail
	if(_SETTINGS->video.enabled)
		_burger->resetTrail();

	_episodeStepCount = 0;
	_burger->scan();// initial observation
	_episodeState = RUNNING;
	_episodeTime = 0.f;
	_totalReward = 0.f;
}

void Environment::BeginContact(b2Contact * contact){
	if(_episodeState != RUNNING)// already episode over -> goal reached, nothing to check
		return;

	b2Fixture * a = contact->GetFixtureA();
	b2Fixture * b = contact->GetFixtureB();
	const b2Fixture * burger = _burger->getSafetyRadiusSensor();
	if(_level != NULL){
		b2Fixture * goal = _level->getGoal();
		b2Fixture * other_fix = NULL;
		if(a == burger){
			other_fix = b;
		}
		else if(b == burger){
			other_fix = a;
		}
		if(other_fix != NULL){
			// end of episode
			if(other_fix == goal){// goal reached
				_reward += _SETTINGS->training.reward_goal;
				_episodeState = POSITIVE_END;
			}
			else if(_burger->beginContact()){// wall hit
				_reward += _SETTINGS->training.reward_wall;
				if(_SETTINGS->training.episode_over_on_hit)
					_episodeState = NEGATIVE_END;
			}
		}
	}
}

void Environment::EndContact(b2Contact * contact)
{
	const b2Fixture * burger_sensor = _burger->getSafetyRadiusSensor();
	if(burger_sensor == contact->GetFixtureA() || burger_sensor == contact->GetFixtureB()){
		_burger->endContact();
	}
}

void Environment::render(const Camera & cam, const zRect & aabb)
{
	// set colorplex shader
	_RENDERER->useColorplexShader();
	_RENDERER->setGLMatrix();
	cam.upload();
	_RENDERER->resetModelviewMatrix();

	// update burger trail
	_burger->updateTrail();

	_PHYSICS->calculateVisibleFixturesWorld(_world, aabb);
	// rendering goal spawn area
	if(_SETTINGS->gui.show_goal_spawn && _level != NULL){
		_level->renderGoalSpawn();
	}

	if(_SETTINGS->gui.show_trail)
		_burger->renderTrail();
	
	uint16 category = 0x0001;
	if(_SETTINGS->gui.show_robot)
		category |= COLLIDE_CATEGORY_PLAYER; 

	_PHYSICS->setDynamicColor(zColor(0x3A87E1FF));
	_PHYSICS->debugDraw(PHYSICS_RENDER_ALL, category);
	if(_SETTINGS->gui.show_stage){
		_PHYSICS->setDynamicColor(zColor(0x5ACB79FF));
		_PHYSICS->debugDrawWorld(_world, PHYSICS_RENDER_ALL, COLLIDE_CATEGORY_STAGE | COLLIDE_CATEGORY_GOAL);
	}

	// use color shader
	_RENDERER->useColorShader();
	_RENDERER->setGLMatrix();
	cam.upload();
	_RENDERER->resetModelviewMatrix();

	//draw laser scanner
	if(_SETTINGS->gui.show_laser){
		_burger->renderScan(_SETTINGS->gui.show_laser > 1);
	}
}
