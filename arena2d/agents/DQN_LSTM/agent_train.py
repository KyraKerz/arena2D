import time
import fc_lstm
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter
from torch.nn.utils.rnn import pack_sequence
import numpy
import random
from collections import deque

### hyper parameters ###
MEAN_REWARD_BOUND = 120.0	# training is considered to be done if the mean reward reaches this value
NUM_ACTIONS = 6				# total number of discrete actions the robot can perform
DISCOUNT_FACTOR = 0.99		# discount factor for reward estimation (often denoted by gamma)
SYNC_TARGET_STEPS = 2000	# target net is synchronized with net every X steps
LEARNING_RATE = 0.00025 	# learning rate for optimizer
EPSILON_START = 1			# start value of epsilon
EPSILON_MAX_STEPS = 10**6	# how many steps until epsilon reaches minimum
EPSILON_END = 0.02			# min epsilon value
BATCH_SIZE = 64				# batch size for training after every step
TRAINING_START = 1000		# start training only after the first X steps
MEMORY_SIZE = 2**20			# last X states will be stored in a buffer (memory), from which the batches are sampled
NUM_INPUTS = 362			# number of elements in observation passed by arena
N_STEPS = 2
DOUBLE = True
#######################

AGENT_NAME="dqn_lstm_agent"

class Agent:
	def initialize(self, device, model_name, agent_name):
		self.device = device
		self.agent_name = agent_name;

		# creating xp buffers on gpu for faster sampling
		self.tensor_state_buffer = torch.zeros(MEMORY_SIZE, NUM_INPUTS ,dtype=torch.float).to(device)# state
		# rewards with applied N-Step: buffer[t] = reward_t + discount*buffer[t+1] + ... discount^(N-1)*buffer[t+N-1]
		self.tensor_reward_buffer = torch.zeros(MEMORY_SIZE, dtype=torch.float).to(device)
		self.tensor_action_buffer = torch.zeros(MEMORY_SIZE, dtype=torch.long).to(device)# the action that was chosen
		self.tensor_done_buffer = torch.zeros(MEMORY_SIZE, dtype=torch.bool).to(device)# episode has ended
		self.tensor_step_buffer = torch.zeros(MEMORY_SIZE, dtype=torch.int16).to(device)# step index in episode (starting at 0)

		# creating net and target net
		self.net = fc_lstm.FC_LSTM(NUM_INPUTS, NUM_ACTIONS)
		self.tgt_net = fc_lstm.FC_LSTM(NUM_INPUTS, NUM_ACTIONS)

		# copy to device
		self.net.to(self.device)
		self.tgt_net.to(self.device)

		# showing net to user
		print(self.net)

		# creating writer
		self.writer = SummaryWriter(comment="-"+self.agent_name)
		self.episode_start = 0

		# load model parameters from file if given
		if model_name is not None:
			self.net.load_state_dict(torch.load(model_name))
			self.tgt_net.load_state_dict(self.net.state_dict())

		# initialize epsilon for epsilon-greedy algorithm
		self.epsilon = EPSILON_START

		# create optimizer
		self.optimizer = optim.Adam(self.net.parameters(), lr=LEARNING_RATE)

		# mean rewards
		self.best_mean_reward = None
		self.mean_reward = 0

		self.new_state_discount = DISCOUNT_FACTOR**N_STEPS
		# time metrics
		self.sampling_times = deque(maxlen=100)
		self.batch_forward_times =deque(maxlen=100)
		self.loss_calc_times = deque(maxlen=100)
		self.backward_times = deque(maxlen=100)
		self.optimize_times = deque(maxlen=100)
		self.gpu_pre_copy_times = deque(maxlen=100)
		self.gpu_copy_times = deque(maxlen=100)
		self.measure_gpu_times = False

		# training metrics
		self.mean_loss_buffer = deque(maxlen=100)
		self.mean_value_buffer = deque(maxlen=100)
		self.mean_loss = 0
		self.mean_value = 0

		# initializing frame indicies
		self.frame_idx = 0
		self.last_episode_frame = 0
		self.episode_idx = self.episode_start
		self.training_done = False

		# getting current time
		self.last_time = time.perf_counter()
		self.start_time = time.time();

		# reset state
		self.reset()

	# calculate epsilon according to current frame index
	def epsilon_decay(self):
		self.epsilon = max(EPSILON_END, EPSILON_START - self.frame_idx/float(EPSILON_MAX_STEPS))

	# reset collected reward
	def reset(self):
		self.total_reward = 0.0
		self.episode_frames = 0
		self.last_observation = None
	
	def pre_step(self, observation):
		# measuring gpu times only every 100 frames (better performance)
		self.measure_gpu_times = (self.frame_idx%100 == 0)
		self.last_observation = observation
		self.epsilon_decay()
		self.start_gpu_measure()
		action = -1
		# insert current state into buffer
		idx = self.frame_idx%MEMORY_SIZE
		self.tensor_state_buffer[idx] = torch.FloatTensor(observation);
		self.tensor_step_buffer[idx] = self.episode_frames
		self.stop_gpu_measure(self.gpu_pre_copy_times)
		if random.random() <= self.epsilon: # random action
			action = random.randrange(0, NUM_ACTIONS)
		else:
			# pack sequence
			sequence = self.pack_episodes([idx])
			q = self.net(sequence)
			max_value, act_v = torch.max(q, dim=1)
			self.mean_value_buffer.append(max_value.item())
			action = int(act_v.item())

		return action
	
	def pack_episodes(self, indicies):
		sequence_list = []
		for i in indicies:
			sequence = None
			episode_step_index = int(self.tensor_step_buffer[i])
			start_index = i-episode_step_index
			if start_index < 0: # wrap around -> two part sequence
				# sequence part 1
				seq1 = torch.narrow(self.tensor_state_buffer, dim=0, start=int(MEMORY_SIZE+start_index), length=-start_index)
				# sequence part 2
				seq2 = torch.narrow(self.tensor_state_buffer, dim=0, start=0, length=i+1)
				sequence = torch.cat((seq1, seq2), 0)
			else:# continuous sequence 
				sequence = torch.narrow(self.tensor_state_buffer, dim=0, start=int(start_index), length=episode_step_index+1)
			# add sequence to list
			sequence_list.append(sequence)
		# packing all together
		return pack_sequence(sequence_list, enforce_sorted=False)

	
	def post_step(self, new_observation, action, reward, mean_reward, is_done, mean_success):
		self.start_gpu_measure()
		idx = self.frame_idx%MEMORY_SIZE
		if is_done: # save next state if done, because next pre_step will have different state
			self.tensor_state_buffer[(idx+1)%MEMORY_SIZE] = torch.FloatTensor(new_observation)
		self.tensor_reward_buffer[idx] = reward
		self.tensor_action_buffer[idx] = action
		self.tensor_done_buffer[idx] = (is_done != 0)
		# update reward from last n steps
		max_steps = min(N_STEPS, self.episode_frames+1)
		discount = 1
		for i in range(1, max_steps):
			discount *= DISCOUNT_FACTOR
			pre_idx = (MEMORY_SIZE+idx-i)%MEMORY_SIZE
			self.tensor_reward_buffer[pre_idx] += reward * discount
		# set done of last n steps to true if episode has ended
		if is_done != 0:
			for i in range(1, max_steps):
				pre_idx = (MEMORY_SIZE+idx-i)%MEMORY_SIZE
				self.tensor_done_buffer[pre_idx] = True

		# stop gpu measure
		self.stop_gpu_measure(self.gpu_copy_times)

		# calculate metrics
		self.mean_reward = mean_reward
		self.total_reward += reward
		self.mean_success = mean_success
		if len(self.mean_value_buffer) > 0:
			self.mean_value = numpy.mean(list(self.mean_value_buffer))
		if len(self.mean_loss_buffer) > 0:
			self.mean_loss = numpy.mean(list(self.mean_loss_buffer))

		# count total frames/episode frames
		self.frame_idx += 1
		self.episode_frames += 1

		if is_done != 0: # episode done
			self.episode_idx += 1 # increment episode count

			# check if new best mean
			self.check_best_mean()

			# writing metrics
			self.write_stats()

			# check whether mean reward bound reached
			if mean_reward >= MEAN_REWARD_BOUND and (self.episode_idx-self.episode_start) >= 100:
				print("Solved in %d episodes (%d frames)!"%(self.episode_idx, self.frame_idx))
				self.save_model_weights("%s_final.dat"%(self.agent_name));
				self.training_done = True
				return 1 # stop training 

		# only optimize if enough data in buffer
		if self.frame_idx >= TRAINING_START:
			# set weights in target net from training net every SYNC_TARGET_STEPS frames
			if self.frame_idx % SYNC_TARGET_STEPS == 0:
				self.tgt_net.load_state_dict(self.net.state_dict())

			# optimizing weights through SGD
			self.optimizer.zero_grad()
			self.start_gpu_measure()
			batch = self.sample(BATCH_SIZE)
			self.stop_gpu_measure(self.sampling_times)

			loss_t = self.calc_loss(*batch)
			
			self.start_gpu_measure()
			loss_t.backward()
			self.stop_gpu_measure(self.backward_times)

			self.start_gpu_measure()
			t_before = time.perf_counter()
			self.optimizer.step()
			self.stop_gpu_measure(self.optimize_times)

		if is_done != 0:
			# reset episode
			self.reset()

		return 0 # continue training

	def write_stats(self):
		# calculate and write metrix
		speed = (self.frame_idx - self.last_episode_frame)/(time.perf_counter() - self.last_time)
		self.last_episode_frame = self.frame_idx
		mean_sampling = "-"
		mean_loss_calc = "-"
		mean_optimize = "-"
		mean_batch = "-"
		mean_backward = "-"
		mean_gpu_copy = "-"
		best_reward = "-"
		memory_size = min(self.frame_idx, MEMORY_SIZE)
		if len(self.gpu_copy_times) > 0:
			mean_gpu_copy = "%.3fms"%((numpy.mean(list(self.gpu_copy_times)) + numpy.mean(list(self.gpu_pre_copy_times)))*1000)
		if len(self.loss_calc_times) > 0:
			mean_loss_calc = "%.3fms"%(numpy.mean(list(self.loss_calc_times))*1000)
		if len(self.optimize_times) > 0:
			mean_optimize = "%.3fms"%(numpy.mean(list(self.optimize_times))*1000)
		if len(self.sampling_times) > 0:
			mean_sampling = "%.3fms"%(numpy.mean(list(self.sampling_times))*1000)
		if len(self.batch_forward_times) > 0:
			mean_batch = "%.3fms"%(numpy.mean(list(self.batch_forward_times))*1000)
		if len(self.backward_times) > 0:
			mean_backward = "%.3fms"%(numpy.mean(list(self.backward_times))*1000)
		if self.best_mean_reward is not None:
			best_reward = "%.3f"%(self.best_mean_reward)
		print("  ----- Agent -----")
		print("  Times:")
		print("   -> GPU copy:         "+mean_gpu_copy)
		print("   -> Batch sampling:   "+mean_sampling)
		print("   -> Batch forwarding: "+mean_batch)
		print("   -> Loss calculation: "+mean_loss_calc)
		print("   -> Loss backward:    "+mean_backward)
		print("   -> Optimizing:       "+mean_optimize)
		print("  Best reward: "+best_reward)
		print("  Epsilon:     %.2f"%(self.epsilon))
		print("  Frames:      %d"%(self.frame_idx))
		print("  Memory:      %.1f%% (%d/%d)"%(float(100*memory_size)/float(MEMORY_SIZE), memory_size, MEMORY_SIZE))
		print("  Mean loss:   %.3f"%(self.mean_loss))
		print("  Mean value:  %.3f"%(self.mean_value))
		# plotting
		self.writer.add_scalar("epsilon", self.epsilon, self.episode_idx)
		self.writer.add_scalar("speed", speed, self.episode_idx)
		self.writer.add_scalar("reward", self.total_reward, self.episode_idx)
		self.writer.add_scalar("mean reward", self.mean_reward, self.episode_idx)
		self.writer.add_scalar("mean success", self.mean_success, self.episode_idx)
		self.writer.add_scalar("mean value", self.mean_value, self.episode_idx)
		self.writer.add_scalar("mean loss", self.mean_loss, self.episode_idx)
		# reset timer
		self.last_time = time.perf_counter()
		
	
	# check if new best mean and if reached boundary
	def check_best_mean(self):
		if (self.best_mean_reward is None or self.best_mean_reward < self.mean_reward) and (self.episode_idx-self.episode_start) >= 100:
			self.save_model_weights(self.agent_name + "_best.dat")
			self.best_mean_reward = self.mean_reward
	
	def save_model_weights(self, filename):
		print("***Saving model weights to %s***"%(filename))
		torch.save(self.net.state_dict(), filename)
		with open("%s.txt"%(filename), "w") as f:
			f.write("Episode:       %d\n"%(self.episode_idx))
			f.write("Start episode: %d\n"%(self.episode_start))
			f.write("Frames:        %d\n"%(self.frame_idx))
			f.write("Mean reward:   %f\n"%(self.mean_reward))
			f.write("Mean success:  %f\n"%(self.mean_success))
			f.write("Epsilon:       %f\n"%(self.epsilon))
			delta_time = int(time.time()-self.start_time);
			f.write("Duration:      %dh %dmin %dsec\n"%(delta_time/3600, (delta_time/60)%60, (delta_time%60)))
			f.write(str(self.net))
	
	def start_gpu_measure(self):
		if self.measure_gpu_times:
			if self.tensor_state_buffer.is_cuda:
				torch.cuda.synchronize(self.device)
			self.time_before = time.perf_counter()

	def stop_gpu_measure(self, mean_buffer):
		if self.measure_gpu_times:
			if self.tensor_state_buffer.is_cuda:
				torch.cuda.synchronize(self.device)
			time_after = time.perf_counter()
			mean_buffer.append(time_after-self.time_before)

	# returns batch of size @batch_size as tuple with random samples from xp buffers
	# (state, new_state, action, reward, is_done)
	def sample(self, batch_size):
		# sample random elements
		random_indicies = None
		if self.frame_idx <= MEMORY_SIZE: # buffer not overflowed yet
			random_indicies = numpy.random.choice(self.frame_idx-N_STEPS, batch_size)
		else:
			forbidden_lower_i = (MEMORY_SIZE + self.frame_idx-N_STEPS)%MEMORY_SIZE
			forbidden_upper_i = (MEMORY_SIZE + self.frame_idx-1)%MEMORY_SIZE
			random_indicies = numpy.empty(batch_size, dtype=numpy.long)
			if forbidden_lower_i > forbidden_upper_i: # wrap around
				for i in range(0, batch_size): 
					x = numpy.random.choice(MEMORY_SIZE-N_STEPS) + forbidden_upper_i +1
					random_indicies[i] = x
			else: # no wrap around
				for i in range(0, batch_size): 
					x = numpy.random.choice(MEMORY_SIZE-N_STEPS)
					if x >= forbidden_lower_i:
						x += N_STEPS
					random_indicies[i] = x

		# sample next state indicies of random states
		current_idx = self.frame_idx%MEMORY_SIZE
		random_indicies_next = []
		for i in random_indicies:
			random_indicies_next.append((i+N_STEPS)%MEMORY_SIZE)

		# copy indicies to gpu for faster sampling
		random_indicies_v = torch.tensor(random_indicies).to(self.device)

		# get actual tensors from indicies
		state = self.pack_episodes(random_indicies)
		new_state = self.pack_episodes(random_indicies_next)
		action = self.tensor_action_buffer[random_indicies_v]
		reward = self.tensor_reward_buffer[random_indicies_v]
		is_done = self.tensor_done_buffer[random_indicies_v]
		return (state, new_state, action, reward, is_done)

	def calc_loss(self, states, next_states, actions, rewards, dones):
		self.start_gpu_measure()
		# get currently estimated action values of performed actions in the past
		q_values = self.net(states);
		state_action_values = q_values.gather(1, actions.unsqueeze(-1)).squeeze(-1)
		# estimate action values of next states
		if DOUBLE:# double dqn
			next_state_actions = self.net(next_states).max(1)[1]
			next_state_values = self.tgt_net(next_states).gather(1, next_state_actions.unsqueeze(-1)).squeeze(-1)
		else:
			next_state_values = self.tgt_net(next_states).max(1)[0]
		# set estimated state values of done states to 0
		next_state_values[dones] = 0.0
		# detatch from flow graph
		next_state_values = next_state_values.detach()
		# Bellman
		expected_state_action_values = next_state_values * self.new_state_discount + rewards
		self.stop_gpu_measure(self.batch_forward_times)

		# loss function
		self.start_gpu_measure()
		l = nn.MSELoss()(state_action_values, expected_state_action_values)
		self.stop_gpu_measure(self.loss_calc_times)
		self.mean_loss_buffer.append(l.item())
		return l

	def close(self):
		# shutting down writer
		self.writer.close()
		if not self.training_done:
			agent.save_model_weights("%s_episode%d.dat"%(self.agent_name, self.episode_idx))

# global agent instance
agent = Agent()

### init ###
# called before training is started
# device_name: "cpu", "cuda", etc.
# model_name: name of model file to load as initial weights or None if no initial model given
# continue_training: if set to True, training continues from last episode
def init(device_name="cpu", model_name=None, continue_training=False):
	# get device from name
	device = torch.device(device_name)

	print("initializing agent...")
	# initialize agent
	agent.initialize(device, model_name, AGENT_NAME)


### pre_step ###
# this function is called before the simulation step
# return the action to perform
def pre_step(observation):
	# update agent and return action to perform
	return agent.pre_step(observation)

### post_step ###
# this function is called after simulation step has been performed
# return 0 to continue, 1 to stop training
def post_step(new_observation, action, reward, mean_reward, is_done, mean_success):

	# update agent and return whether to continue or stop training
	c = agent.post_step(new_observation, action, reward, mean_reward, is_done, mean_success)
	return c

### stop ###
# this function is called when training is stopped
def stop():
	agent.close()