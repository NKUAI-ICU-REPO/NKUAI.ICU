import torch.nn.functional

from base_model import *
import gym
from copy import deepcopy


class DDPG:

    def __init__(self, state_dim, hidden_size, action_dim, max_action, env:gym.Env, episodes, batch_size, buffer_size, lr=0.01, gamma=0.9, tau=0.02):
        self.env = env
        self.episodes = episodes
        self.replay_buffer = Replay_Buffer(buffer_size, batch_size)
        self.gamma = gamma
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.actor = Actor(state_dim, hidden_size, action_dim, max_action)
        self.actor_target = Actor(state_dim, hidden_size, action_dim, max_action)
        self.actor_target.load_state_dict(self.actor.state_dict())
        self.critic = Critic(state_dim, hidden_size, action_dim)
        self.critic_target = Critic(state_dim, hidden_size, action_dim)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.optim_for_actor = torch.optim.Adam(self.actor.parameters(), lr=lr)
        self.optim_for_critic = torch.optim.Adam(self.critic.parameters(), lr=lr)
        self.tau = tau

    def soft_update(self, network: nn.Module, target_network: nn.Module):
        for param, target_param in zip(network.parameters(), target_network.parameters()):
            target_param.data.copy_(self.tau*param.data + (1-self.tau)*target_param.data)

    def choose_action(self, state: np.ndarray):
        state_torch = torch.from_numpy(state).float()
        action = self.actor(state_torch).squeeze()
        return float(action.detach().numpy())
    
    def learn(self):
        counters = []
        for episode in range(self.episodes):
            counter = 0
            done = False
            observation = self.env.reset()
            while not done:
                counter += 1
                action = self.choose_action(observation)
                observation_, reward, done, _ = self.env.step(round(action))
                self.replay_buffer.push(deepcopy(observation), action, reward, deepcopy(observation_), done)
                observation = observation_
                if not self.replay_buffer.is_full():
                    continue
                train_s, train_a, train_r, train_s_, train_done = self.replay_buffer.sample()
                train_s = torch.Tensor(train_s).view(-1, self.state_dim)
                train_a = torch.Tensor(train_a).view(-1, self.action_dim)
                train_r = torch.Tensor(train_r).view(-1, 1)
                train_s_ = torch.Tensor(train_s_).view(-1, self.state_dim)
                train_done = torch.Tensor(train_done).view(-1, 1)
                q_pre = self.critic(train_s,train_a)
                q_target = self.critic_target(train_s_,self.actor_target(train_s))
                q_target = train_r + self.gamma*q_target*(1-train_done)
                loss_for_critic = torch.nn.functional.mse_loss(q_pre,q_target.detach())
                loss_for_critic.backward()
                self.optim_for_critic.step()
                loss_for_actor = -(self.critic(train_s,self.actor(train_s))).mean()
                self.optim_for_actor.zero_grad()
                loss_for_actor.backward()
                self.optim_for_actor.step()
                self.soft_update(self.actor, self.actor_target)
                self.soft_update(self.critic, self.critic_target)
            counters.append(counter)
            print("episode {} end, stay for {} steps".format(episode+1, counter))
            if len(counters)>5 and counters[-5]==200 and counters[-4]==200 and counters[-3]==200 and counters[-2]==200 and counters[-1]==200:
                return counters
        return counters