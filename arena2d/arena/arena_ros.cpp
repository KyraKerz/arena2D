#include "arena.h"

#ifdef ARENA_USE_ROS
int Arena::initROS(int argc, char *argv[])
{
	// init ros without sigint handler (SDL has its own)
	ros::init(argc, argv, ROS_NODE_NAME, ros::init_options::NoSigintHandler);
	ros::NodeHandle n;
	INFO("-> advertising laser scan");
	_laserPub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);
	if (!_laserPub)
	{
		return -1;
	}
	INFO("-> advertising distance to goal");
	_distancePub = n.advertise<geometry_msgs::Vector3>("distance", 1000);
	if (!_distancePub)
	{
		return -1;
	}

	INFO("-> advertising command service");
	_commandService = n.advertiseService("arena_command", &Arena::remoteCommand, this);
	if (!_commandService)
	{
		return -1;
	}
	// use another node handle to set up a speperate CallbackQueue
	ros::NodeHandle nh_interaction;
	nh_interaction.setCallbackQueue(&interaction_callback_queue_);
	INFO("-> advertising interaction service");
	interaction_server_ = nh_interaction.advertiseService<Arena, arena2d::InteractionDiscActs::Request, arena2d::InteractionDiscActs::Response>(
		"area/interaction", &Arena::interactionCallback, this);
	if (!interaction_server_)
	{
		return -1;
	}
	// by using async spinner the process invoke it's start() and stop() will not get blocked.
	interaction_async_spinner_ptr_ = std::unique_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1, &interaction_callback_queue_));
	return 0;
}

bool Arena::remoteCommand(arena2d::arenaCommand::Request &req,
						  arena2d::arenaCommand::Response &res)
{
	INFO_F("Remote command: '%s'", req.command.c_str());
	res.result = (int)command(req.command.c_str());

	return true;
}

bool Arena::interactionCallback(arena2d::InteractionDiscActs::Request &req,
								arena2d::InteractionDiscActs::Response &res)
{

	// _remoteStep = true;
	// int action = static_cast<int>(req.action);
	// if (action == -1) // reset level
	// {
	// 	reset();
	// }
	// else if (action < static_cast<int>(Burger::NUM_ACTIONS)) // physics step
	// {
	// 	physicsStep((Burger::Action)action);
	// }
	// else // invalid action
	// {
	// 	ROS_ERROR("Invalid action ('%d')!", action);
	// 	return false;
	// }

	// // collecting observations and reward
	// getObservationMessage(res.obs);
	// res.reward = _reward;
	// // check episode over
	// res.over = _episodeEnd;

	// return true;
	// is all actions are negative and same, them special mode should be toggled.
	// pthread_mutex_lock(&interaction_mutex_);
	while(m_need_perform_actions){};
	bool enter_special_mode = false;
	if (req.actions.size() != this->_numEnvs)
	{
		std::stringstream ss;
		ss << " the number of environments" << _numEnvs << "  provided by arena2d doesn't match the number of actions"
		   << req.actions.size() << " given by agent.";
		throw std::logic_error(ss.str().c_str());
		ERROR_F("the number of environments (%d) provided by arena2d doesn't match the number of actions (%d) given by agent.",
				_numEnvs, static_cast<int>(req.actions.size()));
		cmdStopTraining(0, NULL);
		return false;
	}
	for (size_t i = 0; i < _numEnvs; i++)
	{
		if (req.actions[i] > _numEnvs)
		{
			ERROR_F("the value of action is out of range actions[%d]: %d ", (int)i, req.actions[i]);
			cmdStopTraining(0, NULL);
			return false;
		}
		if (req.actions[i] < 0)
		{
			INFO("Agent sent stopping SIG, training finished");
			cmdStopTraining(0, nullptr);
			return false;
		}
		_actions[i] = req.actions[i];
	}
	m_need_perform_actions = true;
	while(m_need_perform_actions){};
	static uint32_t seq = 0;
	//packing response message
	auto t = ros::Time::now();
	for (size_t env_idx = 0; env_idx < _numEnvs; env_idx++)
	{
		int num_beam;
		const float *laser_data = _envs[env_idx].getScan(num_beam);
		// set laser scan
		arena2d::InteractionDiscActsResponse::_obs_type::value_type obs;
		// arena2d::InteractionDiscActsResponse::_rewards_type rewards;
		// arena2d::InteractionDiscActsResponse::_episodes_done_type dones;
		auto &laser_scan = obs.laser;
		laser_scan.header.frame_id = "laser_scan";
		laser_scan.header.seq = seq++;
		laser_scan.header.stamp = t;
		laser_scan.angle_increment = 360 / num_beam;
		laser_scan.angle_max = 360.f;
		laser_scan.angle_min = 0.f;
		for (int i = 0; i < num_beam; i++)
		{
			laser_scan.intensities.push_back(laser_data[i]);
		}
		// x is distance , y is angle
		float angle = 0,distance = 0;
		_envs[env_idx].getGoalDistance(distance,angle);	
		obs.relativ_goal_pos.x = distance;
		obs.relativ_goal_pos.y = angle;
		res.obs.push_back(obs);
		res.rewards.push_back(_envs[env_idx].getReward());
		res.episodes_done.push_back(_dones[env_idx]);
	}
	
	// pthread_mutex_unlock(&interaction_mutex_);
	return true;
}

void Arena::ros_run()
{
	_updateTimer.reset();
	_physicsTimer.reset();
	_videoTimer.reset();
	int iteration = 1;
	int next_video_update_it = 1; // iteration at which to perform the next video update
	int remainder = 0;
	m_need_perform_actions = false;
	// pthread_mutex_init(&interaction_mutex_, nullptr);
	interaction_async_spinner_ptr_->start();
	if (_SETTINGS->video.enabled)
	{ /* video */
		while (_run)
		{
			// if fps is limit, we lock the mutex for synchronization with the thread running interaction service.
			if (!(_trainingMode && _videoDisabled))
			{
				// pthread_mutex_lock(&interaction_mutex_);
			}
			_updateTimer.setTargetFPS(_SETTINGS->physics.fps);
			_videoTimer.setTargetFPS(_SETTINGS->video.fps);
			_physicsTimer.setTargetFPS(_SETTINGS->physics.fps);

			if (iteration == next_video_update_it)
			{
				zEventList evt;
				if (Z_FW->update(&evt) == 0) /* user closed window */
					exitApplication();

				if (_consoleEnabled)
					updateConsole(evt);
				processEvents(evt);

				if (!_videoDisabled)
				{
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
				if (_SETTINGS->video.fps < update_fps)
				{
					int f = remainder + update_fps;
					next_video_update_it = f / _SETTINGS->video.fps;
					remainder = (f % _SETTINGS->video.fps);
				}
				// measure fps
				_videoTimer.update(false);
			}
			step_update();
			int d = _updateTimer.update(!(_trainingMode && _videoDisabled)); // no fps limit if in training mode and video disabled
			if (!(_trainingMode && _videoDisabled))
			{
				// pthread_mutex_unlock(&interaction_mutex_);
				// given the thread running interaction service enough time to get the mutex.
				// std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			iteration++;
		}
	}
	else
	{ /* no video */
		while (_run)
		{
			_updateTimer.setTargetFPS(_SETTINGS->physics.fps);
			_physicsTimer.setTargetFPS(_SETTINGS->physics.fps);
			step_update();
			_updateTimer.update(!_trainingMode); // no fps limit when in training mode
		}
	}
}
void Arena::step_update(){
	if(! m_need_perform_actions){
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
		if(action != -1){
			for(int i = 0; i < _numEnvs; i++){
				_actions[i] = action;
				m_need_perform_actions = true;
			}
		}
	}
	if(m_need_perform_actions){
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
				INFO("done");
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
		m_need_perform_actions = false;


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
}

#endif
