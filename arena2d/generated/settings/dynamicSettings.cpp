//THIS FILE IS AUTO-GENERATED! DO NOT EDIT!

#include <engine/globalSettings.h>

void GlobalSettings::initSymbolTable()
{
	_hashTable = h_init(97, GlobalSettings::stringHash, NULL);
	sSettingsOption * option = NULL;

	//video.resolution_w
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.video.resolution_w);
	h_add(_hashTable, "video.resolution_w", 19, option, sizeof(sSettingsOption));

	//video.resolution_h
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.video.resolution_h);
	h_add(_hashTable, "video.resolution_h", 19, option, sizeof(sSettingsOption));

	//video.window_x
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.video.window_x);
	h_add(_hashTable, "video.window_x", 15, option, sizeof(sSettingsOption));

	//video.window_y
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.video.window_y);
	h_add(_hashTable, "video.window_y", 15, option, sizeof(sSettingsOption));

	//video.maximized
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.video.maximized);
	h_add(_hashTable, "video.maximized", 16, option, sizeof(sSettingsOption));

	//video.msaa
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.video.msaa);
	h_add(_hashTable, "video.msaa", 11, option, sizeof(sSettingsOption));

	//video.vsync
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.video.vsync);
	h_add(_hashTable, "video.vsync", 12, option, sizeof(sSettingsOption));

	//video.fps
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.video.fps);
	h_add(_hashTable, "video.fps", 10, option, sizeof(sSettingsOption));

	//video.fullscreen
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.video.fullscreen);
	h_add(_hashTable, "video.fullscreen", 17, option, sizeof(sSettingsOption));

	//video.enabled
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.video.enabled);
	h_add(_hashTable, "video.enabled", 14, option, sizeof(sSettingsOption));

	//gui.font_size
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.gui.font_size);
	h_add(_hashTable, "gui.font_size", 14, option, sizeof(sSettingsOption));

	//gui.show_robot
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.gui.show_robot);
	h_add(_hashTable, "gui.show_robot", 15, option, sizeof(sSettingsOption));

	//gui.show_stage
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.gui.show_stage);
	h_add(_hashTable, "gui.show_stage", 15, option, sizeof(sSettingsOption));

	//gui.show_laser
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.gui.show_laser);
	h_add(_hashTable, "gui.show_laser", 15, option, sizeof(sSettingsOption));

	//gui.show_stats
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.gui.show_stats);
	h_add(_hashTable, "gui.show_stats", 15, option, sizeof(sSettingsOption));

	//gui.show_goal_spawn
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.gui.show_goal_spawn);
	h_add(_hashTable, "gui.show_goal_spawn", 20, option, sizeof(sSettingsOption));

	//gui.show_trail
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.gui.show_trail);
	h_add(_hashTable, "gui.show_trail", 15, option, sizeof(sSettingsOption));

	//gui.camera_follow
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.gui.camera_follow);
	h_add(_hashTable, "gui.camera_follow", 18, option, sizeof(sSettingsOption));

	//gui.camera_x
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.gui.camera_x);
	h_add(_hashTable, "gui.camera_x", 13, option, sizeof(sSettingsOption));

	//gui.camera_y
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.gui.camera_y);
	h_add(_hashTable, "gui.camera_y", 13, option, sizeof(sSettingsOption));

	//gui.camera_zoom
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.gui.camera_zoom);
	h_add(_hashTable, "gui.camera_zoom", 16, option, sizeof(sSettingsOption));

	//gui.camera_rotation
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.gui.camera_rotation);
	h_add(_hashTable, "gui.camera_rotation", 20, option, sizeof(sSettingsOption));

	//gui.camera_zoom_factor
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.gui.camera_zoom_factor);
	h_add(_hashTable, "gui.camera_zoom_factor", 23, option, sizeof(sSettingsOption));

	//keys.up
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_KEY;
	option->data = &(_settings.keys.up);
	h_add(_hashTable, "keys.up", 8, option, sizeof(sSettingsOption));

	//keys.left
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_KEY;
	option->data = &(_settings.keys.left);
	h_add(_hashTable, "keys.left", 10, option, sizeof(sSettingsOption));

	//keys.down
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_KEY;
	option->data = &(_settings.keys.down);
	h_add(_hashTable, "keys.down", 10, option, sizeof(sSettingsOption));

	//keys.right
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_KEY;
	option->data = &(_settings.keys.right);
	h_add(_hashTable, "keys.right", 11, option, sizeof(sSettingsOption));

	//keys.reset
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_KEY;
	option->data = &(_settings.keys.reset);
	h_add(_hashTable, "keys.reset", 11, option, sizeof(sSettingsOption));

	//physics.time_step
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.physics.time_step);
	h_add(_hashTable, "physics.time_step", 18, option, sizeof(sSettingsOption));

	//physics.step_iterations
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.physics.step_iterations);
	h_add(_hashTable, "physics.step_iterations", 24, option, sizeof(sSettingsOption));

	//physics.fps
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.physics.fps);
	h_add(_hashTable, "physics.fps", 12, option, sizeof(sSettingsOption));

	//physics.position_iterations
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.physics.position_iterations);
	h_add(_hashTable, "physics.position_iterations", 28, option, sizeof(sSettingsOption));

	//physics.velocity_iterations
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.physics.velocity_iterations);
	h_add(_hashTable, "physics.velocity_iterations", 28, option, sizeof(sSettingsOption));

	//training.max_time
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.training.max_time);
	h_add(_hashTable, "training.max_time", 18, option, sizeof(sSettingsOption));

	//training.episode_over_on_hit
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.training.episode_over_on_hit);
	h_add(_hashTable, "training.episode_over_on_hit", 29, option, sizeof(sSettingsOption));

	//training.reward_goal
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.training.reward_goal);
	h_add(_hashTable, "training.reward_goal", 21, option, sizeof(sSettingsOption));

	//training.reward_distance_to_goal_decreased
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.training.reward_distance_to_goal_decreased);
	h_add(_hashTable, "training.reward_distance_to_goal_decreased", 43, option, sizeof(sSettingsOption));

	//training.reward_distance_to_goal_increased
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.training.reward_distance_to_goal_increased);
	h_add(_hashTable, "training.reward_distance_to_goal_increased", 43, option, sizeof(sSettingsOption));

	//training.reward_wall
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.training.reward_wall);
	h_add(_hashTable, "training.reward_wall", 21, option, sizeof(sSettingsOption));

	//training.reward_time_out
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.training.reward_time_out);
	h_add(_hashTable, "training.reward_time_out", 25, option, sizeof(sSettingsOption));

	//training.num_envs
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.training.num_envs);
	h_add(_hashTable, "training.num_envs", 18, option, sizeof(sSettingsOption));

	//training.num_threads
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.training.num_threads);
	h_add(_hashTable, "training.num_threads", 21, option, sizeof(sSettingsOption));

	//robot.laser_noise
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.laser_noise);
	h_add(_hashTable, "robot.laser_noise", 18, option, sizeof(sSettingsOption));

	//robot.laser_max_distance
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.laser_max_distance);
	h_add(_hashTable, "robot.laser_max_distance", 25, option, sizeof(sSettingsOption));

	//robot.laser_start_angle
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.laser_start_angle);
	h_add(_hashTable, "robot.laser_start_angle", 24, option, sizeof(sSettingsOption));

	//robot.laser_end_angle
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.laser_end_angle);
	h_add(_hashTable, "robot.laser_end_angle", 22, option, sizeof(sSettingsOption));

	//robot.laser_num_samples
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.robot.laser_num_samples);
	h_add(_hashTable, "robot.laser_num_samples", 24, option, sizeof(sSettingsOption));

	//robot.forward_speed.linear
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.forward_speed.linear);
	h_add(_hashTable, "robot.forward_speed.linear", 27, option, sizeof(sSettingsOption));

	//robot.forward_speed.angular
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.forward_speed.angular);
	h_add(_hashTable, "robot.forward_speed.angular", 28, option, sizeof(sSettingsOption));

	//robot.left_speed.linear
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.left_speed.linear);
	h_add(_hashTable, "robot.left_speed.linear", 24, option, sizeof(sSettingsOption));

	//robot.left_speed.angular
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.left_speed.angular);
	h_add(_hashTable, "robot.left_speed.angular", 25, option, sizeof(sSettingsOption));

	//robot.right_speed.linear
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.right_speed.linear);
	h_add(_hashTable, "robot.right_speed.linear", 25, option, sizeof(sSettingsOption));

	//robot.right_speed.angular
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.right_speed.angular);
	h_add(_hashTable, "robot.right_speed.angular", 26, option, sizeof(sSettingsOption));

	//robot.strong_left_speed.linear
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.strong_left_speed.linear);
	h_add(_hashTable, "robot.strong_left_speed.linear", 31, option, sizeof(sSettingsOption));

	//robot.strong_left_speed.angular
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.strong_left_speed.angular);
	h_add(_hashTable, "robot.strong_left_speed.angular", 32, option, sizeof(sSettingsOption));

	//robot.strong_right_speed.linear
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.strong_right_speed.linear);
	h_add(_hashTable, "robot.strong_right_speed.linear", 32, option, sizeof(sSettingsOption));

	//robot.strong_right_speed.angular
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.strong_right_speed.angular);
	h_add(_hashTable, "robot.strong_right_speed.angular", 33, option, sizeof(sSettingsOption));

	//robot.backward_speed.linear
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.backward_speed.linear);
	h_add(_hashTable, "robot.backward_speed.linear", 28, option, sizeof(sSettingsOption));

	//robot.backward_speed.angular
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.robot.backward_speed.angular);
	h_add(_hashTable, "robot.backward_speed.angular", 29, option, sizeof(sSettingsOption));

	//stage.initial_level
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.stage.initial_level);
	h_add(_hashTable, "stage.initial_level", 20, option, sizeof(sSettingsOption));

	//stage.level_size
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.stage.level_size);
	h_add(_hashTable, "stage.level_size", 17, option, sizeof(sSettingsOption));

	//stage.max_obstacle_size
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.stage.max_obstacle_size);
	h_add(_hashTable, "stage.max_obstacle_size", 24, option, sizeof(sSettingsOption));

	//stage.min_obstacle_size
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.stage.min_obstacle_size);
	h_add(_hashTable, "stage.min_obstacle_size", 24, option, sizeof(sSettingsOption));

	//stage.num_obstacles
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.stage.num_obstacles);
	h_add(_hashTable, "stage.num_obstacles", 20, option, sizeof(sSettingsOption));

	//stage.dynamic_obstacle_size
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.stage.dynamic_obstacle_size);
	h_add(_hashTable, "stage.dynamic_obstacle_size", 28, option, sizeof(sSettingsOption));

	//stage.num_dynamic_obstacles
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.stage.num_dynamic_obstacles);
	h_add(_hashTable, "stage.num_dynamic_obstacles", 28, option, sizeof(sSettingsOption));

	//stage.obstacle_speed
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.stage.obstacle_speed);
	h_add(_hashTable, "stage.obstacle_speed", 21, option, sizeof(sSettingsOption));

	//stage.enable_position_reset
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.stage.enable_position_reset);
	h_add(_hashTable, "stage.enable_position_reset", 28, option, sizeof(sSettingsOption));

	//stage.goal_size
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_FLOAT;
	option->data = &(_settings.stage.goal_size);
	h_add(_hashTable, "stage.goal_size", 16, option, sizeof(sSettingsOption));

	//stage.svg_path
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_STRING;
	option->data = _settings.stage.svg_path;
	h_add(_hashTable, "stage.svg_path", 15, option, sizeof(sSettingsOption));

	//ros.enabled
	option = (sSettingsOption*)malloc(sizeof(sSettingsOption));
	option->type = SETTINGS_TYPE_INT;
	option->data = &(_settings.ros.enabled);
	h_add(_hashTable, "ros.enabled", 12, option, sizeof(sSettingsOption));

}

void GlobalSettings::writeToFile(FILE * f)
{
	char float_buffer[64];

	fprintf(f, "video{\n");
	fprintf(f, "\tresolution_w = %d\n", _settings.video.resolution_w);
	fprintf(f, "\tresolution_h = %d\n", _settings.video.resolution_h);
	fprintf(f, "\twindow_x = %d\n", _settings.video.window_x);
	fprintf(f, "\twindow_y = %d\n", _settings.video.window_y);
	fprintf(f, "\tmaximized = %d\n", _settings.video.maximized);
	fprintf(f, "\tmsaa = %d\n", _settings.video.msaa);
	fprintf(f, "\tvsync = %d\n", _settings.video.vsync);
	fprintf(f, "\tfps = %d\n", _settings.video.fps);
	fprintf(f, "\tfullscreen = %d\n", _settings.video.fullscreen);
	fprintf(f, "\tenabled = %d\n", _settings.video.enabled);
	fprintf(f, "}\n\n");
	fprintf(f, "gui{\n");
	fprintf(f, "\tfont_size = %d\n", _settings.gui.font_size);
	fprintf(f, "\tshow_robot = %d\n", _settings.gui.show_robot);
	fprintf(f, "\tshow_stage = %d\n", _settings.gui.show_stage);
	fprintf(f, "\tshow_laser = %d\n", _settings.gui.show_laser);
	fprintf(f, "\tshow_stats = %d\n", _settings.gui.show_stats);
	fprintf(f, "\tshow_goal_spawn = %d\n", _settings.gui.show_goal_spawn);
	fprintf(f, "\tshow_trail = %d\n", _settings.gui.show_trail);
	fprintf(f, "\tcamera_follow = %d\n", _settings.gui.camera_follow);
	zStringTools::fromFloat(_settings.gui.camera_x, float_buffer);
	fprintf(f, "\tcamera_x = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.gui.camera_y, float_buffer);
	fprintf(f, "\tcamera_y = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.gui.camera_zoom, float_buffer);
	fprintf(f, "\tcamera_zoom = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.gui.camera_rotation, float_buffer);
	fprintf(f, "\tcamera_rotation = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.gui.camera_zoom_factor, float_buffer);
	fprintf(f, "\tcamera_zoom_factor = %s\n", float_buffer);
	fprintf(f, "}\n\n");
	fprintf(f, "keys{\n");
	fprintf(f, "\tup = %s\n", getSettingsKeyName(_settings.keys.up));
	fprintf(f, "\tleft = %s\n", getSettingsKeyName(_settings.keys.left));
	fprintf(f, "\tdown = %s\n", getSettingsKeyName(_settings.keys.down));
	fprintf(f, "\tright = %s\n", getSettingsKeyName(_settings.keys.right));
	fprintf(f, "\treset = %s\n", getSettingsKeyName(_settings.keys.reset));
	fprintf(f, "}\n\n");
	fprintf(f, "physics{\n");
	zStringTools::fromFloat(_settings.physics.time_step, float_buffer);
	fprintf(f, "\ttime_step = %s\n", float_buffer);
	fprintf(f, "\tstep_iterations = %d\n", _settings.physics.step_iterations);
	fprintf(f, "\tfps = %d\n", _settings.physics.fps);
	fprintf(f, "\tposition_iterations = %d\n", _settings.physics.position_iterations);
	fprintf(f, "\tvelocity_iterations = %d\n", _settings.physics.velocity_iterations);
	fprintf(f, "}\n\n");
	fprintf(f, "training{\n");
	zStringTools::fromFloat(_settings.training.max_time, float_buffer);
	fprintf(f, "\tmax_time = %s\n", float_buffer);
	fprintf(f, "\tepisode_over_on_hit = %d\n", _settings.training.episode_over_on_hit);
	zStringTools::fromFloat(_settings.training.reward_goal, float_buffer);
	fprintf(f, "\treward_goal = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.training.reward_distance_to_goal_decreased, float_buffer);
	fprintf(f, "\treward_distance_to_goal_decreased = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.training.reward_distance_to_goal_increased, float_buffer);
	fprintf(f, "\treward_distance_to_goal_increased = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.training.reward_wall, float_buffer);
	fprintf(f, "\treward_wall = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.training.reward_time_out, float_buffer);
	fprintf(f, "\treward_time_out = %s\n", float_buffer);
	fprintf(f, "\tnum_envs = %d\n", _settings.training.num_envs);
	fprintf(f, "\tnum_threads = %d\n", _settings.training.num_threads);
	fprintf(f, "}\n\n");
	fprintf(f, "robot{\n");
	zStringTools::fromFloat(_settings.robot.laser_noise, float_buffer);
	fprintf(f, "\tlaser_noise = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.robot.laser_max_distance, float_buffer);
	fprintf(f, "\tlaser_max_distance = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.robot.laser_start_angle, float_buffer);
	fprintf(f, "\tlaser_start_angle = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.robot.laser_end_angle, float_buffer);
	fprintf(f, "\tlaser_end_angle = %s\n", float_buffer);
	fprintf(f, "\tlaser_num_samples = %d\n", _settings.robot.laser_num_samples);
	fprintf(f, "\tforward_speed{\n");
	zStringTools::fromFloat(_settings.robot.forward_speed.linear, float_buffer);
	fprintf(f, "\t\tlinear = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.robot.forward_speed.angular, float_buffer);
	fprintf(f, "\t\tangular = %s\n", float_buffer);
	fprintf(f, "\t}\n");
	fprintf(f, "\tleft_speed{\n");
	zStringTools::fromFloat(_settings.robot.left_speed.linear, float_buffer);
	fprintf(f, "\t\tlinear = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.robot.left_speed.angular, float_buffer);
	fprintf(f, "\t\tangular = %s\n", float_buffer);
	fprintf(f, "\t}\n");
	fprintf(f, "\tright_speed{\n");
	zStringTools::fromFloat(_settings.robot.right_speed.linear, float_buffer);
	fprintf(f, "\t\tlinear = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.robot.right_speed.angular, float_buffer);
	fprintf(f, "\t\tangular = %s\n", float_buffer);
	fprintf(f, "\t}\n");
	fprintf(f, "\tstrong_left_speed{\n");
	zStringTools::fromFloat(_settings.robot.strong_left_speed.linear, float_buffer);
	fprintf(f, "\t\tlinear = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.robot.strong_left_speed.angular, float_buffer);
	fprintf(f, "\t\tangular = %s\n", float_buffer);
	fprintf(f, "\t}\n");
	fprintf(f, "\tstrong_right_speed{\n");
	zStringTools::fromFloat(_settings.robot.strong_right_speed.linear, float_buffer);
	fprintf(f, "\t\tlinear = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.robot.strong_right_speed.angular, float_buffer);
	fprintf(f, "\t\tangular = %s\n", float_buffer);
	fprintf(f, "\t}\n");
	fprintf(f, "\tbackward_speed{\n");
	zStringTools::fromFloat(_settings.robot.backward_speed.linear, float_buffer);
	fprintf(f, "\t\tlinear = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.robot.backward_speed.angular, float_buffer);
	fprintf(f, "\t\tangular = %s\n", float_buffer);
	fprintf(f, "\t}\n");
	fprintf(f, "}\n\n");
	fprintf(f, "stage{\n");
	fprintf(f, "\tinitial_level = %d\n", _settings.stage.initial_level);
	zStringTools::fromFloat(_settings.stage.level_size, float_buffer);
	fprintf(f, "\tlevel_size = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.stage.max_obstacle_size, float_buffer);
	fprintf(f, "\tmax_obstacle_size = %s\n", float_buffer);
	zStringTools::fromFloat(_settings.stage.min_obstacle_size, float_buffer);
	fprintf(f, "\tmin_obstacle_size = %s\n", float_buffer);
	fprintf(f, "\tnum_obstacles = %d\n", _settings.stage.num_obstacles);
	zStringTools::fromFloat(_settings.stage.dynamic_obstacle_size, float_buffer);
	fprintf(f, "\tdynamic_obstacle_size = %s\n", float_buffer);
	fprintf(f, "\tnum_dynamic_obstacles = %d\n", _settings.stage.num_dynamic_obstacles);
	zStringTools::fromFloat(_settings.stage.obstacle_speed, float_buffer);
	fprintf(f, "\tobstacle_speed = %s\n", float_buffer);
	fprintf(f, "\tenable_position_reset = %d\n", _settings.stage.enable_position_reset);
	zStringTools::fromFloat(_settings.stage.goal_size, float_buffer);
	fprintf(f, "\tgoal_size = %s\n", float_buffer);
	fprintf(f, "\tsvg_path = \"%s\"\n", _settings.stage.svg_path);
	fprintf(f, "}\n\n");
	fprintf(f, "ros{\n");
	fprintf(f, "\tenabled = %d\n", _settings.ros.enabled);
	fprintf(f, "}\n\n");
}
