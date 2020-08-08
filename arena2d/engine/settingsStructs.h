//NOTE: This file will be loaded by the settings loader, do not add any definitions, nor #includes except for typedef struct and basic variables

typedef struct{
	int resolution_w;// window width
	int resolution_h;// window height
	int window_x;// window x position on startup -1 for centered
	int window_y;// window y position on startup -1 for centered
	int maximized;// start window maximized 1/0
	int msaa;// multisampling anti-aliasing 2,4,8,16
	int vsync;// vertical synchronization 1/0
	int fps; // video frames per second
	int fullscreen; // fullscreen enabled 1/0
	int enabled; // video mode enabled 1/0; if set to 0
}f_videoSettings;

typedef struct{
	// key mapping for controlling the robot
	SDL_Keycode up;
	SDL_Keycode left;
	SDL_Keycode down;
	SDL_Keycode right;
	SDL_Keycode reset;// key for resetting robot
}f_keymapSettings;

typedef struct{
	float time_step; // physics time step
	int step_iterations; // how often to perform a physics update per step
	int fps; // how many times per second a simulation step is performed (also determines number of ros spins per second)
	int position_iterations; // position iterations for each time step (higher value increases simulation accuracy)
	int velocity_iterations; // velocity iterations for each time step (higher value increases simulation accuracy)
}f_physicsSettings;

typedef struct{
	int initial_level; // level loaded on startup (-1 for none)
	float level_size; // width and height of default levels
	float max_obstacle_size; // maximum diameter of static obstacles
	float min_obstacle_size; // minimum diameter of static obstacles
	int num_obstacles; // number of static obstacles
	float dynamic_obstacle_size; //size of dynamic obstacle
	int num_dynamic_obstacles; // number of dynamic obstacles in static_dynamic level
	float obstacle_speed; //in m/s for dynamic obstacles
	int enable_position_reset; // if set to 1 robot's position will be set to the center of the stage after every episode;	if set to 0 robot's position will only be reset when hitting a wall
	float goal_size; // diameter of circular goal to reach
	char svg_path[256]; // path to folder where svg files are stored
}f_stage;

typedef struct{
	float max_time; // maximum time per episode (actual time, so physics.time_step influences maximum number of steps per episode)
	int episode_over_on_hit; // if set to 1 episode ends if an obstacle is hit
	float reward_goal;// reward for reaching goal
	float reward_distance_to_goal_decreased;// reward when distance to goal decreases
	float reward_distance_to_goal_increased;// reward when distance to goal increases
	float reward_wall;// reward for hitting wall
	float reward_time_out; // reward when episode timed out (after max_time seconds)
	int num_envs; //number of parallel environments
	int num_threads;// number of threads to run in parallel, if set to -1 number of cpu cores will be detected automatically
}f_trainingSettings;

typedef struct{
	float linear;
	float angular;
} f_twist;

typedef struct{
	float laser_noise; // random, uniformly distributed offset with a maximum of +/- laser_noise*distance_measured (a value of 0 means perfect laser data -> no noise)
	float laser_max_distance; 	// maximum distance the laser can recognize
	float laser_start_angle; 	// angle in degree of first sample
	float laser_end_angle; 		// angle in degree of last sample
	int laser_num_samples;		// number of laser samples
	// velocities
	f_twist forward_speed;
	f_twist left_speed;
	f_twist right_speed;
	f_twist strong_left_speed;
	f_twist strong_right_speed;
	f_twist backward_speed;
}f_robotSettings;

typedef struct{
	int enabled; // when set to 1, the application is registered as a ros node, certain services and publishers will be created
}f_rosSettings;

typedef struct{
	int font_size; // gui font size
	int show_robot; // show/hide the robot
	int show_stage; // show/hide the stage
	int show_laser; // show/hide the laser
	int show_stats; // show/hide the statistics (e.g. #episodes)
	int show_goal_spawn; // show/hide the spawn area for the goal
	int show_trail; // show/hide robot trail
	int camera_follow;// if == 1 camera follows robot, (if == 2, rotation is also taken into acount)
	float camera_x; // initial position of camera
	float camera_y; // initial position of camera
	float camera_zoom; // initial zoom of camera ('zoom' actually means scaling of the view -> value < 1 means zooming out)
	float camera_rotation; // view rotation in degree
	float camera_zoom_factor; // how does scaling increase/decrease with each zoom step
}f_guiSettings;

//global settings struct
typedef struct{
	f_videoSettings video;
	f_guiSettings gui;
	f_keymapSettings keys;
	f_physicsSettings physics;
	f_trainingSettings training;
	f_robotSettings robot;
	f_stage stage;
	f_rosSettings ros;
}f_settings;
