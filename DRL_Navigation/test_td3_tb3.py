import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from td3_env import TD3Env

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()
        self.layer_1 = nn.Linear(state_dim, 800)
        self.layer_2 = nn.Linear(800, 600)
        self.layer_3 = nn.Linear(600, action_dim)
        self.tanh = nn.Tanh()
    def forward(self, s):
        s = F.relu(self.layer_1(s))
        s = F.relu(self.layer_2(s))
        a = self.tanh(self.layer_3(s))
        return a

class TD3(object):
    def __init__(self, state_dim, action_dim):
        self.actor = Actor(state_dim, action_dim).to(device)
    def get_action(self, state):
        state = torch.Tensor(state.reshape(1, -1)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()
    def load(self, filename, directory):
        self.actor.load_state_dict(torch.load(f"{directory}/{filename}_actor.pth", map_location=device))

# ==== Parameters (edit as needed) ====
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
seed = 0
max_ep = 500
file_name = "TD3_tb3"
launchfile = "abc.launch"  # Change to your launchfile if needed

environment_dim = 24
robot_dim = 4

env = TD3Env(launchfile=launchfile, environment_dim=environment_dim)
time.sleep(5)
torch.manual_seed(seed)
np.random.seed(seed)
state_dim = environment_dim + robot_dim
action_dim = 2

network = TD3(state_dim, action_dim)
try:
    network.load(file_name, "./model_weights")
    print(f"Loaded actor model from ./model_weights/{file_name}_actor.pth")
except Exception as e:
    raise ValueError(f"Could not load the stored model parameters: {e}")

done = False
episode_timesteps = 0
episode_num = 0
state = env.reset()
episode_reward = 0

print("---- Starting Simulation/Evaluation ----")
while True:
    action = network.get_action(np.array(state))
    a_in = [(action[0] + 1) / 2, action[1]]
    next_state, reward, done, target = env.step(a_in)
    episode_reward += reward
    print(f"Ep {episode_num} | Step {episode_timesteps} | Reward: {reward:.2f} | Target: {target} | Done: {done}")
    episode_timesteps += 1

    if done or (episode_timesteps >= max_ep):
        print(f"== Episode {episode_num} finished after {episode_timesteps} steps | Total Reward: {episode_reward:.2f} ==")
        state = env.reset()
        done = False
        episode_timesteps = 0
        episode_num += 1
        episode_reward = 0
    else:
        state = next_state
