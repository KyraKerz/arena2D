video{
	resolution_w = 800
	resolution_h = 600
	window_x = -1
	window_y = -1
	maximized = 0
	msaa = 0
	vsync = 0
	fps = 60
	fullscreen = 0
	enabled = 1
}

gui{
	font_size = 14
	show_robot = 1
	show_stage = 1
	show_laser = 1
	show_stats = 1
	show_goal_spawn = 0
	show_trail = 1
	camera_follow = 0
	camera_x = 0.0
	camera_y = 0.0
	camera_zoom = 0.45
	camera_rotation = 0.0
	camera_zoom_factor = 1.3
}

keys{
	up = KEY_UP
	left = KEY_LEFT
	down = KEY_DOWN
	right = KEY_RIGHT
	reset = KEY_r
}

physics{
	time_step = 0.1
	step_iterations = 5
	fps = 60
	position_iterations = 8
	velocity_iterations = 12
}

training{
	max_time = 60.0
	episode_over_on_hit = 0
	reward_goal = 100.0
	reward_distance_to_goal_decreased = 0.1
	reward_distance_to_goal_increased = -0.2
	reward_wall = -10.0
	reward_time_out = 0.0
	num_envs = 4
	num_threads = -1
}

robot{
	laser_noise = 0.0
	laser_max_distance = 3.5
	laser_start_angle = 0.0
	laser_end_angle = 359.0
	laser_num_samples = 360
	forward_speed{
		linear = 0.2
		angular = 0.0
	}
	left_speed{
		linear = 0.15
		angular = 0.75
	}
	right_speed{
		linear = 0.15
		angular = -0.75
	}
	strong_left_speed{
		linear = 0.0
		angular = 1.5
	}
	strong_right_speed{
		linear = 0.0
		angular = -1.5
	}
	backward_speed{
		linear = -0.1
		angular = 0.0
	}
}

stage{
	initial_level = 5
	level_size = 4.0
	max_obstacle_size = 1.0
	min_obstacle_size = 0.1
	num_obstacles = 8
	dynamic_obstacle_size = 0.3
	num_dynamic_obstacles = 4
	obstacle_speed = 0.08
	enable_position_reset = 0
	goal_size = 0.1
	svg_path = "level/examples/"
}

ros{
	enabled = 1
}

