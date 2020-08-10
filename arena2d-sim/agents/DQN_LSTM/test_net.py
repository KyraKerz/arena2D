import agent_train

agent_train.init()
obs = [0]*362
for i in range(5):
	action = agent_train.pre_step(obs)
	obs = [i+1]*362
	agent_train.post_step(obs, action, i, 0, False, 0)

