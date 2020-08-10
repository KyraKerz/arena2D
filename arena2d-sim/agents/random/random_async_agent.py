import random

NUM_ACTIONS = 6

### init ###
# called once before training starts
# @device_name string defining the device to use for training ('cpu' or 'cuda')
# @model_name name of model to load from file and to initialize the net with, set to None if not specified by user
# @num_envs number of parallel environments in the simulator


def init(device_name, model_name, num_envs):
    global NUM_ENVS
    NUM_ENVS = num_envs
    pass

### pre_step ###
# this function is called before the simulation step
# @observations is a list containing the observations (distance, angle, laser0...laserN-1) from each environment
# return a list of actions that are performed in the environments


def pre_step(observations):
    global NUM_ENVS
    actions = []
    for i in range(NUM_ENVS):
        actions.append(random.randrange(NUM_ACTIONS))

    return actions

### post_step ###
# this function is called after simulation step has been performed
# @new_observations is a list containing the new observations (distance, angle, laser0...laserN-1) from each environment
# @actions are the actions that have been performed in each environment
# @rewards are the rewards that have been received in each environment
# @dones is a list containing value 1 for environments that have completed an episode in this step, else the value is 0
# @mean_reward scalar value, mean reward from last 100 episodes accross all environments
# @mean_success scalar value, mean success rate from last 100 episodes accross all environments
# return 0 to continue, 1 to stop training


def post_step(new_observations, actions, rewards, mean_reward, dones, mean_success):
    return 0

### stop ###
# called when training has been stopped by the user in the simulator


def stop():
    pass
