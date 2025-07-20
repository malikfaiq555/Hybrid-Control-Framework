import os
import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from numpy import inf
from torch.utils.tensorboard import SummaryWriter

from replay_buffer import ReplayBuffer
from td3_env import TD3Env

# ==== TD3 Actor and Critic Network Definitions ====

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

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()
        self.layer_1 = nn.Linear(state_dim, 800)
        self.layer_2_s = nn.Linear(800, 600)
        self.layer_2_a = nn.Linear(action_dim, 600)
        self.layer_3 = nn.Linear(600, 1)
        self.layer_4 = nn.Linear(state_dim, 800)
        self.layer_5_s = nn.Linear(800, 600)
        self.layer_5_a = nn.Linear(action_dim, 600)
        self.layer_6 = nn.Linear(600, 1)
    def forward(self, s, a):
        s1 = F.relu(self.layer_1(s))
        s11 = torch.mm(s1, self.layer_2_s.weight.data.t())
        s12 = torch.mm(a, self.layer_2_a.weight.data.t())
        s1 = F.relu(s11 + s12 + self.layer_2_a.bias.data)
        q1 = self.layer_3(s1)
        s2 = F.relu(self.layer_4(s))
        s21 = torch.mm(s2, self.layer_5_s.weight.data.t())
        s22 = torch.mm(a, self.layer_5_a.weight.data.t())
        s2 = F.relu(s21 + s22 + self.layer_5_a.bias.data)
        q2 = self.layer_6(s2)
        return q1, q2

# ==== TD3 Algorithm Wrapper ====

class TD3(object):
    def __init__(self, state_dim, action_dim, max_action):
        self.actor = Actor(state_dim, action_dim).to(device)
        self.actor_target = Actor(state_dim, action_dim).to(device)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters())
        self.critic = Critic(state_dim, action_dim).to(device)
        self.critic_target = Critic(state_dim, action_dim).to(device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters())
        self.max_action = max_action
        self.writer = SummaryWriter()
        self.iter_count = 0

    def get_action(self, state):
        state = torch.Tensor(state.reshape(1, -1)).to(device)
        return self.actor(state).cpu().data.numpy().flatten()

    def train(self, replay_buffer, iterations, batch_size=100, discount=0.99,
              tau=0.005, policy_noise=0.2, noise_clip=0.5, policy_freq=2):
        av_Q, max_Q, av_loss = 0, -inf, 0
        for it in range(iterations):
            batch_states, batch_actions, batch_rewards, batch_dones, batch_next_states = replay_buffer.sample_batch(batch_size)
            state = torch.Tensor(batch_states).to(device)
            next_state = torch.Tensor(batch_next_states).to(device)
            action = torch.Tensor(batch_actions).to(device)
            reward = torch.Tensor(batch_rewards).to(device)
            done = torch.Tensor(batch_dones).to(device)

            # Target policy smoothing
            next_action = self.actor_target(next_state)
            noise = torch.Tensor(batch_actions).data.normal_(0, policy_noise).to(device)
            noise = noise.clamp(-noise_clip, noise_clip)
            next_action = (next_action + noise).clamp(-self.max_action, self.max_action)

            # TD3: Double Q
            target_Q1, target_Q2 = self.critic_target(next_state, next_action)
            target_Q = torch.min(target_Q1, target_Q2)
            av_Q += torch.mean(target_Q)
            max_Q = max(max_Q, torch.max(target_Q))
            target_Q = reward + ((1 - done) * discount * target_Q).detach()

            current_Q1, current_Q2 = self.critic(state, action)
            loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)
            self.critic_optimizer.zero_grad()
            loss.backward()
            self.critic_optimizer.step()

            # Delayed policy updates
            if it % policy_freq == 0:
                actor_grad, _ = self.critic(state, self.actor(state))
                actor_grad = -actor_grad.mean()
                self.actor_optimizer.zero_grad()
                actor_grad.backward()
                self.actor_optimizer.step()
                # Polyak averaging update for target networks
                for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
                    target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)
                for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
                    target_param.data.copy_(tau * param.data + (1 - tau) * target_param.data)
            av_loss += loss

        self.iter_count += 1
        self.writer.add_scalar("loss", av_loss / iterations, self.iter_count)
        self.writer.add_scalar("Av. Q", av_Q / iterations, self.iter_count)
        self.writer.add_scalar("Max. Q", max_Q, self.iter_count)

    def save(self, filename, directory):
        torch.save(self.actor.state_dict(), f"{directory}/{filename}_actor.pth")
        torch.save(self.critic.state_dict(), f"{directory}/{filename}_critic.pth")

    def load(self, filename, directory):
        self.actor.load_state_dict(torch.load(f"{directory}/{filename}_actor.pth"))
        self.critic.load_state_dict(torch.load(f"{directory}/{filename}_critic.pth"))

# ==== Evaluation Function ====

def evaluate(env, network, epoch, eval_episodes=10):
    avg_reward, col = 0.0, 0
    rewards_per_episode = []
    for ep in range(eval_episodes):
        count = 0
        state = env.reset()
        done = False
        episode_reward = 0
        while not done and count < 501:
            action = network.get_action(np.array(state))
            a_in = [(action[0] + 1) / 2, action[1]]
            state, reward, done, _ = env.step(a_in)
            episode_reward += reward
            count += 1
            if reward < -90:
                col += 1
        rewards_per_episode.append(episode_reward)
    avg_reward = np.mean(rewards_per_episode)
    avg_col = col / eval_episodes
    print("-------------------------------------------------------------")
    print(f"Epoch {epoch}:")
    print(f"  Average Total Reward over {eval_episodes} Episodes: {avg_reward:.2f}")
    print(f"  Episode Rewards: {[round(r,2) for r in rewards_per_episode]}")
    print(f"  Avg. Collisions per Episode: {avg_col:.2f}")
    print("-------------------------------------------------------------")
    return avg_reward

# ==== Hyperparameters and Initialization ====

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
seed = 0
eval_freq = 5e3
max_ep = 500
eval_ep = 10
max_timesteps = int(5e6)
expl_noise = 1
expl_decay_steps = 500000
expl_min = 0.1
batch_size = 40
discount = 0.99999
tau = 0.005
policy_noise = 0.2
noise_clip = 0.5
policy_freq = 2
buffer_size = int(1e6)
file_name = "TD3_tb3"
save_model = True
load_model = False
random_near_obstacle = True

os.makedirs("./results", exist_ok=True)
if save_model:
    os.makedirs("./model_weights", exist_ok=True)

launchfile = "abc.launch"   # <-- Set your launch file here!
environment_dim = 24
robot_dim = 4
env = TD3Env(launchfile=launchfile, environment_dim=environment_dim)
time.sleep(5)
torch.manual_seed(seed)
np.random.seed(seed)
state_dim = environment_dim + robot_dim
action_dim = 2

network = TD3(state_dim, action_dim, max_action=1)
replay_buffer = ReplayBuffer(buffer_size, seed)

if load_model:
    try:
        network.load(file_name, "./model_weights")
    except Exception as e:
        print(f"Could not load model, initializing new: {e}")

evaluations = []
timestep = 0
timesteps_since_eval = 0
episode_num = 0
done = True
epoch = 1
count_rand_actions = 0
random_action = []

# ==== Main Training Loop ====

while timestep < max_timesteps:
    if done:
        if timestep != 0:
            network.train(
                replay_buffer,
                episode_timesteps,
                batch_size,
                discount,
                tau,
                policy_noise,
                noise_clip,
                policy_freq,
            )
        if timesteps_since_eval >= eval_freq:
            print("Validating (Evaluation)...")
            timesteps_since_eval %= eval_freq
            evaluations.append(
                evaluate(env, network=network, epoch=epoch, eval_episodes=eval_ep)
            )
            network.save(file_name, directory="./model_weights")
            np.save(f"./results/{file_name}", evaluations)
            epoch += 1
        state = env.reset()
        done = False
        episode_reward = 0
        episode_timesteps = 0
        episode_num += 1
    if expl_noise > expl_min:
        expl_noise -= ((1 - expl_min) / expl_decay_steps)
    action = network.get_action(np.array(state))
    action = (action + np.random.normal(0, expl_noise, size=action_dim)).clip(-1, 1)
    if random_near_obstacle:
        if (np.random.uniform(0, 1) > 0.85 and min(state[4:-8]) < 0.6 and count_rand_actions < 1):
            count_rand_actions = np.random.randint(8, 15)
            random_action = np.random.uniform(-1, 1, 2)
        if count_rand_actions > 0:
            count_rand_actions -= 1
            action = random_action
            action[0] = -1
    a_in = [(action[0] + 1) / 2, action[1]]
    next_state, reward, done, target = env.step(a_in)
    done_bool = 0 if episode_timesteps + 1 == max_ep else int(done)
    done = 1 if episode_timesteps + 1 == max_ep else int(done)
    episode_reward += reward
    replay_buffer.add(state, action, reward, done_bool, next_state)
    state = next_state
    episode_timesteps += 1
    timestep += 1
    timesteps_since_eval += 1

# ==== Save and Evaluate Once More ====

evaluations.append(evaluate(env, network=network, epoch=epoch, eval_episodes=eval_ep))
if save_model:
    network.save(file_name, directory="./model_weights")
np.save(f"./results/{file_name}", evaluations)
