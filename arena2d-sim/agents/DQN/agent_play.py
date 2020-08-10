from dqn_models import fc 

import torch
import pathlib

NUM_ACTIONS = 7
INPUT_SHAPE = 362

DEFAULT_MODEL = "agents/DQN/best.dat"

# globals
device = None
net = None

def init(device_name, model_name):
	if model_name is None:
		model_name=DEFAULT_MODEL
	global device
	global net
	device = torch.device(device_name)
	#net = cnn_fc.CNN_FC_DQN(NUM_ACTIONS)
	net = fc.FC_DQN(INPUT_SHAPE, NUM_ACTIONS)
	net.train(False)# set training mode to false to deactivate dropout layer
	net.load_state_dict(torch.load(model_name, map_location=device))
	net.to(device)

### pre_step ###
# this function is called before the simulation step
# return the action to perform
def pre_step(observation):
	# passing observation through net
	state_v = torch.FloatTensor([observation]).to(device)
	q_vals_v = net(state_v)
	# select action with max q value
	_, act_v = torch.max(q_vals_v, dim=1)
	action = int(act_v.item())
	# swap backward and stop action (changed in arena2d since training)
	if action == 6:
		action = 5
	elif action == 5:
		action = 6
	return action

### post_step ###
# this function is called after simulation step has been performed
# return 0 to continue, 1 to stop training
def post_step(new_observation, action, reward, mean_reward, is_done, mean_success):
	return 0 # keep on playing

### stop ###
def stop():
	pass	
