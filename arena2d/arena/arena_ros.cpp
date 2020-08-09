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
	interaction_async_spinner_ptr_ = std::unique_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1,&interaction_callback_queue_));
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
	bool enter_special_mode = false;
	if(req.actions.size()!= this->_numEnvs){
		std::stringstream ss;
		ss<<" the number of environments"<<_numEnvs<<"  provided by arena2d doesn't match the number of actions"
			<<req.actions.size()<<" given by agent.";
		throw std::logic_error(ss.str().c_str());
		ERROR_F("the number of environments (%d) provided by arena2d doesn't match the number of actions (%d) given by agent.",
			_numEnvs,static_cast<int>(req.actions.size()));
			cmdStopTraining(0,NULL);
			return false;
	}
	for(size_t i=0; i< _numEnvs; i++){
		if(req.actions[i]>_numEnvs){
			ERROR("the value of action is out of range");
			cmdStopTraining(0,NULL);
			return false;
		}
		if(req.actions[i]<0){
			INFO("Agent sent stopping SIG, training finished");
			cmdStopTraining(0,nullptr);
			return false;
		}
		_actions[i] = req.actions[i];
	}
	// packing response message
	auto t = ros::Time::now();
	for(size_t env_idx = 0; env_idx<_numEnvs; env_idx++){
		int num_beam;
		const float* laser_data = _envs[env_idx].getScan(num_beam);
		auto& curr_laser = res.obs[env_idx].laser;
		auto& curr_goal = res.obs[env_idx].relativ_goal_pos;
		auto& curr_reward = res.rewards[env_idx];
		auto& curr_env_done = res.episodes_done[env_idx];

		curr_laser.header.stamp = t;
		curr_laser.header.frame_id = "laser_scan";
		curr_laser.angle_min = 0;
		curr_laser.angle_max = 360;
		curr_laser.angle_increment = 360/num_beam;
		for(size_t i = 0; i<num_beam; i++){
			curr_laser.ranges.push_back(laser_data[i]);
		}
		
		float angle = 0,distance = 0;
		_envs[env_idx].getGoalDistance(distance,angle);
		curr_goal.x = distance;
		curr_goal.y = angle;
		curr_reward = _envs[env_idx].getReward();
		curr_env_done = _dones[env_idx];
	}
	// for sync with display.
	pthread_mutex_lock(&interaction_mutex_);
	pthread_mutex_unlock(&interaction_mutex_);
	return true;

}

// void Arena::getObservationMessage(arena2d::observation &obs)
// {
// 	/* get laser scan samples */
// 	int num_samples;
// 	const float *data = _burger->getSamples(num_samples);
// 	for (int i = 0; i < num_samples; i++)
// 		obs.laser.ranges.push_back(data[i]);
// 	obs.laser.header.stamp = ros::Time::now();
// 	obs.laser.header.frame_id = "laser_scan";
// 	obs.laser.angle_min = 0;
// 	obs.laser.angle_max = 360;
// 	obs.laser.angle_increment = 360.f / num_samples;
// 	obs.laser.time_increment = 0;
// 	obs.laser.scan_time = 0;
// 	obs.laser.range_min = 0;
// 	obs.laser.range_max = BURGER_LIDAR_MAX_DISTANCE;

// 	/* get relative distance to goal */
// 	obs.goal.x = 0;
// 	obs.goal.y = 0;
// 	obs.goal.z = 0;
// 	float distance, angle;
// 	getGoalDistance(distance, angle);
// 	obs.goal.x = distance;
// 	obs.goal.y = angle;
// }

// void Arena::publishData()
// {
// 	arena2d::observation obs;
// 	getObservationMessage(obs);
// 	/* publish laser scan */
// 	_laserPub.publish(obs.laser);

// 	/* publish distance to goal (vector) */
// 	_distancePub.publish(obs.goal);
// }
void Arena::ros_run(){
	_updateTimer.reset();
	_physicsTimer.reset();
	_videoTimer.reset();
	int iteration = 1;
	int next_video_update_it = 1;// iteration at which to perform the next video update
	int remainder = 0;
	pthread_mutex_t tmp;
	pthread_mutex_init(&interaction_mutex_,nullptr);
	interaction_async_spinner_ptr_->start();
	if(_SETTINGS->video.enabled){/* video */
		while(_run){
			// if fps is limit, we lock the mutex for synchronization with the thread running interaction service.
			if(!(_trainingMode && _videoDisabled)){ 
				pthread_mutex_lock(&interaction_mutex_);
			}
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
			
			int d = _updateTimer.update(!(_trainingMode && _videoDisabled));// no fps limit if in training mode and video disabled
			if(!(_trainingMode && _videoDisabled)){
				pthread_mutex_unlock(&interaction_mutex_);
				// given the thread running interaction service enough time to get the mutex.
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}	
			iteration++;
	
		}
	}
	else{/* no video */
		while(_run){
			_updateTimer.setTargetFPS(_SETTINGS->physics.fps);
			_physicsTimer.setTargetFPS(_SETTINGS->physics.fps);
			_updateTimer.update(!_trainingMode);// no fps limit when in training mode
		}
	}
}



#endif
