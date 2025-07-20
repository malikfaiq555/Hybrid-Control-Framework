import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from real_env import Env

# ==== Actor definition (same as in training) ====
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

# ==== TD3 policy wrapper ====
class TD3(object):
    def __init__(self, state_dim, action_dim):
        self.actor = Actor(state_dim, action_dim).to(device)
    def get_action(self, state):
        state = torch.Tensor(state.reshape(1, -1)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()
    def load(self, filename, directory):
        self.actor.load_state_dict(torch.load(f"{directory}/{filename}_actor.pth", map_location=device))

# ==== Parameters ====
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
seed = 0
file_name = "TD3_tb3"
environment_dim = 24
robot_dim = 4

env = Env(environment_dim)
time.sleep(2)
torch.manual_seed(seed)
np.random.seed(seed)
state_dim = environment_dim + robot_dim
action_dim = 2

network = TD3(state_dim, action_dim)
try:
    network.load(file_name, "../model_weights")  # relative path from scripts/ to model_weights/
    print(f"Loaded actor model from ../model_weights/{file_name}_actor.pth")
except Exception as e:
    raise ValueError(f"Could not load the stored model parameters: {e}")

episode_num = 0
state = env.reset()

print("---- Starting Real Robot Policy Execution ----")
while True:
    action = network.get_action(np.array(state))
    a_in = [(action[0] + 1) / 2, action[1]]
    next_state, done, target = env.step(a_in)

    print(f"Ep {episode_num} | Target reached: {target} | Done: {done}")
    if done:
        env.reached_goal()
        print(f"== Episode {episode_num} finished (Goal reached!) ==")
        state = env.reset()
        episode_num += 1
    else:
        state = next_state
