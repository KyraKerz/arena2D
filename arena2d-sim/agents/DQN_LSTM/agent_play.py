import fc_lstm
import torch

NUM_ACTIONS = 6
INPUT_SHAPE = 362

# globals
device = None
net = None
last_h = None
last_c = None

def init(device_name, model_name):
	#if model_name is None:
		#raise ValueError("No model given!")
	global device, net, last_h, last_c
	device = torch.device(device_name)
	#net = cnn_fc.CNN_FC_DQN(NUM_ACTIONS)
	net = fc_lstm.FC_LSTM(INPUT_SHAPE, NUM_ACTIONS)
	net.train(False)# set training mode to false to deactivate dropout layer
	net.load_state_dict(torch.load(model_name, map_location=device))
	net.to(device)
	reset_hidden()

### pre_step ###
# this function is called before the simulation step
# return the action to perform
def pre_step(observation):
	global last_h, last_c
	# passing observation through net
	q, (last_h, last_c) = net.forward_hidden(torch.FloatTensor([observation]).to(device), (last_h, last_c))
	# select action with max q value
	_, act_v = torch.max(q, dim=1)
	action = int(act_v.item())
	return action

### post_step ###
# this function is called after simulation step has been performed
# return 0 to continue, 1 to stop training
def post_step(new_observation, action, reward, mean_reward, is_done, mean_success):
	if is_done:
		reset_hidden()
	return 0 # keep on playing

### stop ###
def stop():
	reset_hidden()
	pass

def reset_hidden():
	global last_h, last_c, device
	(last_h, last_c) = net.getInitialHidden();
	last_h = last_h.to(device)
	last_c = last_c.to(device)
