# Overview
The application *Arena2D* provides tools for simulating and training the *Turtlebot Burger* in a 2D environment. It simulates the motors as well as the 360 degree laser sensor of the robot.
This program was written by Cornelius Marx and proudly integrates the following freely available software components:

* GLEW (http://glew.sourceforge.net/)
* lodepng (https://github.com/lvandeve/lodepng)
* Box2D (https://github.com/erincatto/Box2D)
* SDL2 (http://libsdl.org/)
* Freetype (https://freetype.org/)
* ROS Kinetic (https://www.ros.org/)
* Python (https://www.python.org/)

# Building
**NOTE:** Although most of the libraries needed to compile the application are available for other platforms as well (MacOS, Windows), *Arena2D* is primarily targeted at systems running a Linux operating system and thus was only tested on such. 

To build the application you first need to download and install the following developer libraries:

* SDL2 (```sudo apt-get install libsdl2-dev```)
* Freetype (```sudo apt-get install libfreetype-dev```)
* (optional) ROS Kinetic (follow the instructions on http://wiki.ros.org/kinetic)
* (optional) Python Developer Tools (```pip install python-devtools```)

## With ROS
Move this repository to the ```src/``` folder of your catkin workspace. Then run ```catkin_make``` from the root of your catkin workspace.
When compiling with ROS, the application provides a service that enables external agents to control the robot and to retrieve the sensor data in the simulation.

## Without ROS
Simply run the ```build.sh``` script from this directory.

## With Python
The application is build with the Python interpreter by default.
When compiling with Python, the application can execute external Python scripts during simulation. This allows the user to implement custom agents that can be dynamically executed at runtime.

## Without Python
If you want to compile without the python interpreter, simply add the flag ```-DNO_PYTHON=TRUE``` to the build command.

# Running
If you have compiled the application without ROS simply run ```./build/arena``` from this folder.
Otherwise run ```rosrun arena2d arena```.

## Commandline arguments
There are several commandline arguments that can be passed when launching the application.

* ```--help``` show help
* ```--disable-video``` disable window creation and video rendering (no user input)
* ```--run <command>``` run commands separated by ```;``` e.g. ```--run "level 2; start_training agent.py"```
* ```--logfile <file>``` log output to given file


## Controls
For testing purposes, the robot can be controlled by the user using the arrow keys. Hold down the right mouse button while moving the mouse to pan the view. Use the mouse wheel to zoom in and out.

## Hot Keys
* ```F1``` activate/deactivate in-app console
* ```F3``` activate/deactivate video mode (deactivating video mode can be useful during training to increase performance)
* ```Ctrl+S``` save current settings to default settings file (```settings.st```)
* ```Ctrl+L``` load settings from default settings file (note, that there are some options, that when changed, will only take effect on restart)
* ```R``` reset stage (rebindable in settings file)


## In-App console
Press F1 to toggle the In-App console. Commands can be executed by hitting ```Enter```. Use the up and down keys to browse the command history.
The command ```help``` displays all available commands. Run ```help <command>``` to get more detailed information about a given command.
The In-App console is only available with video mode enabled. Otherwise use the comandline argument ```--run <command>``` to run commands on startup.
Most relevant commands:

* ```level <level_id>```: load level with given id:
	* ```0```: empty level with no borders
	* ```1```: empty level with borders
	* ```2```: static level, randomly placed static obstacles
	* ```3```: dynamic level, randomly moving obstacles
* ```start_training <py-script> [--model <initial model>, --device <cpu/cuda>]```: start training session using given script (see section below for more details)
* ```stop_training```: stop current training session
* ```set <settings-expression>```: adjust current settings, e.g. ```set "physics.time_step = 0.0167"```

## Settings parameters
When running the application for the first time a file named ```settings.st``` will be created, containing the default settings.
There are several parameters that can be adjusted to control the environment, simulation and training process:

* Section ```training```:
	* ```max_time```: maximum time per episode (actual time, so ```physics.time_step``` influences maximum number of steps per episode)
	* ```reward_goal```: reward for reaching goal
	* ```reward_towards_goal```: reward for moving towards goal
	* ```towards_goal_angle```: +/- angle (degree) robots needs to be in with respect to goal to receive ```reward_towards_goal```
	* ```reward_away_from_goal```: reward for moving away from goal
	* ```reward_wall```: reward for hitting wall
	* ```reward_time_out```: reward when episode timed out (after ```max_time``` seconds)
	* ```frame_divider```: the agents last action is performed for k successive steps

* Section ```physics```:
	* ```time_step```: physics time step
	* ```fps```: determines how many times per second a simulation step is performed (```time_step```*```fps``` = RealTimeFactor); ignored when video mode is disabled
	* ```position_iterations```: position iterations for each time step (higher value increases simulation accuracy)
	* ```velocity_iterations```: velocity iterations for each time step (higher value increases simulation accuracy)

* Section ```robot```:
	* ```laser_noise```: uniformly distributed random offset on each laser sample, proportional to the actual distance
	* ```laser_max_distance```: maximum distance that can be measured by laser scanner, distances greater than this value will be clamped
	* ```laser_start_angle```: angle (degree) of first laser sample, (relative to the direction the robot is facing)
	* ```laser_end_angle```: angle (degree) of last laser sample, (relative to the direction the robot is facing)
	* ```laser_num_samples```: number of samples for laser scan
	* ```*_speed```: angular (in rad/s) and linear (in m/s) velocity associated with action (substitute ```*``` with ```left```, ```right```, ```strong_left```, ```string_right```, ```backward```)

## Running agent script (Python only)
When executing ```start_training <py-agent>``` a training session will be started using the callback functions defined in the given python-script. Note that it is not neccessary to implement every callback function in the script.

* ```def init(device_name, model_name):``` function will be called initially and only once per training session
	* ```device_name``` will be passed from the application with ```start_training --device <d>```, default value is "cpu" 
	* ```model_name``` will be passed from the application with ```start_training --model <m>```, variable is ```None``` if not specified

* ```def pre_step(observation):``` function will be called before the simulation step is performed
	* ```observation```: array consisting of ```[DistanceToGoal, AngleToGoal, LaserSample[0], ... , LaserSample[N-1]]```; the angle is measured in degree; ```N``` is the value specified in the settings by ```robot.laser_num_samples```
	* ```return```: integer specifying what action to perform (0:```FORWARD```, 1:```LEFT```, 2:```RIGHT```, 3:```STRONG_LEFT```, 4:```STRONG_RIGHT```, 5:```STOP```, 6:```BACKWARD```)

* ```def post_step(new_observation, action, reward, mean_reward, is_done, mean_success):``` function will be called after the simulation step has been performed
	* ```new_observation```: new observation after action was executed (same format as ```observation``` from ```pre_step```)
	* ```action```: action that has been performed by agent (return value from last call to ```pre_step```)
	* ```reward```: reward gained by performing last action
	* ```mean_reward```: mean reward from last 100 episodes (can be used as bound for stopping training)
	* ```is_done```: 0 if episode has not ended yet; 1 if episode has ended and was successfull (goal reached); -1 if episode has ended and agent failed (time out or obstacle hit)
	* ```mean_success```: mean success rate from last 100 episodes (0 to 1)
	* ```return```: 0 to continue training; 1 to stop training

* ```def stop():```: will be called once training has been stopped by user or agent (return value of function ```post_step```)

For a simple example on how to implement the callbacks, please have a look at [agents/random_agent.py](./agents/random_agent.py).
