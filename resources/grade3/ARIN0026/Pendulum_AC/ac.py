import gym, os
from itertools import count
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Categorical

RENDER = True
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
env = gym.make("CartPole-v0").unwrapped

state_size = env.observation_space.shape[0]
action_size = env.action_space.n
lr = 0.0001

class Actor(nn.Module):
    def __init__(self, state_size, action_size):
        super(Actor, self).__init__()
        self.state_size = state_size
        self.action_size = action_size
        self.linear1 = nn.Linear(self.state_size, 128)
        self.linear2 = nn.Linear(128, 256)
        self.linear3 = nn.Linear(256, self.action_size)

    def forward(self, state):
        output = F.relu(self.linear1(state))
        output = F.relu(self.linear2(output))
        output = self.linear3(output)
        distribution = Categorical(F.softmax(output, dim=-1))
        return distribution


class Critic(nn.Module):
    def __init__(self, state_size, action_size):
        super(Critic, self).__init__()
        self.state_size = state_size
        self.action_size = action_size
        self.linear1 = nn.Linear(self.state_size, 128)
        self.linear2 = nn.Linear(128, 256)
        self.linear3 = nn.Linear(256, 1)

    def forward(self, state):
        output = F.relu(self.linear1(state))
        output = F.relu(self.linear2(output))
        value = self.linear3(output)
        return value


def compute_returns(next_value, rewards, masks, gamma=0.99):
    R = next_value
    returns = []
    for step in reversed(range(len(rewards))):
        R = rewards[step] + gamma * R * masks[step]
        returns.insert(0, R)
    return returns


def trainIters(actor, critic, n_iters):
    optimizerA = optim.Adam(actor.parameters())
    optimizerC = optim.Adam(critic.parameters())
    for iter in range(n_iters):
        state = env.reset()
        log_probs = []
        values = []
        rewards = []
        masks = []
        entropy = 0
        env.reset()
        mx_reward = None

        for i in count():
            if RENDER:
                env.render()
            # -------补充代码--------
            state = torch.FloatTensor(state).to(device)
            dist = actor(state)
            val = critic(state)

            action = dist.sample()
            action = action.cpu().numpy()
            observation_, reward, done, info = env.step(action)
            action = torch.from_numpy(action).to(device)
            log_prob = dist.log_prob(action).unsqueeze(0)
            entropy+=dist.entropy().mean()
            mask = 1-done

            reward = torch.FloatTensor([reward]).to(device)
            mask  = torch.FloatTensor([mask]).to(device)


            values.append(val)
            rewards.append(reward)
            log_probs.append(log_prob)
            masks.append(mask)


            state = observation_


            if done:
                print('max reward from training Q Network is {}'.format(torch.max(rewards).cpu().numpy()))
                print('Iteration: {}, Score: {}'.format(iter, i))
                break

        # ------更新网络------
        state = torch.tensor(state, dtype=torch.float32, device=device)

        get_val = critic(state)

        returns = compute_returns(get_val,rewards,masks)

        log_probs = torch.cat(log_probs)
        values = torch.cat(values)
        returns = torch.cat(returns).detach()

        optim_supervised = returns - values

        actor_loss  = -(log_probs*optim_supervised.detach()).mean()

        critic_loss = torch.pow(optim_supervised,2).mean()


        optimizerA.zero_grad()
        optimizerC.zero_grad()

        actor_loss.backward()

        critic_loss.backward()
        optimizerA.step()
        optimizerC.step()





    if not os.path.exists('./model/'):
        os.mkdir('./model/')

    torch.save(actor, 'model/actor.pkl')
    torch.save(critic, 'model/critic.pkl')
    env.close()


if __name__ == '__main__':
    if os.path.exists('model/actor.pkl'):
        actor = torch.load('model/actor.pkl')
        print('Actor Model loaded')
    else:
        actor = Actor(state_size, action_size).to(device)
    if os.path.exists('model/critic.pkl'):
        critic = torch.load('model/critic.pkl')
        print('Critic Model loaded')
    else:
        critic = Critic(state_size, action_size).to(device)
    trainIters(actor, critic, n_iters=100)