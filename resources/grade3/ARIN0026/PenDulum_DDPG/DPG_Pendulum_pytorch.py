import torch
from torch import utils
import torch.nn.functional as F
import torchvision
from matplotlib import  pyplot as plt
import numpy as np
import gym




render_mode = False


# if torch.cuda.is_available():
#     device = torch.device('cuda')
# else:
#     device = torch.device('cpu')

class Sample():
    def __init__(self,env,policyNet):
        self.env=env
        self.policy_net = policyNet
        self.gamma=0.95

    def sample_episodes(self,num_episode):
        batch_obs = []
        batch_actions =[]
        batch_rwd =[]


        for i in range(num_episode):
            observation = self.env.reset()

            reward_episode = []

            while True:
                state = np.reshape(observation,[1,3])
                action = self.policy_net.choose_action(state)
                # if device == torch.device('cuda'):
                #     action = action.cpu().numpy()
                # else :
                #     action = action.numpy()
                observation_ , reward,done,_ = self.env.step(action)


                batch_obs.append(np.reshape(observation,[1,3])[0,:])
                batch_actions.append(action)
                reward_episode.append((reward+8)/8)

                if done:
                    reward_sum =0
                    discounted_sum_reward = np.zeros_like(reward_episode)
                    for t in reversed(range(len(reward_episode))):
                        reward_sum = reward_sum*self.gamma + reward_episode[t]
                        discounted_sum_reward[t] = reward_sum
                    for t in range(len(reward_episode)) :
                        batch_rwd.append(discounted_sum_reward[t])

                    break

                observation = observation_

        batch_obs=np.reshape(batch_obs,[len(batch_obs),self.policy_net.n_features])
        batch_actions = np.reshape(batch_actions,[len(batch_actions),1])
        batch_rwd = np.reshape(batch_rwd , [len(batch_rwd),1])


        return batch_obs,batch_actions,batch_rwd

class PolicyNet(torch.nn.Module):
    def __init__(self,env):
        super(PolicyNet,self).__init__()
        self.n_features = env.observation_space.shape[0]
        self.n_actions =1
        #self.action_bound =action_bound
        #self.obs = torch.tensor(0,dtype=torch.float32).expand(None,self.n_features)
        self.dist_normal = None


        self.f1 = torch.nn.Linear(in_features=self.n_features,out_features=200)
        torch.nn.init.normal_(self.f1.weight,mean=0,std=0.1)
        torch.nn.init.constant_(self.f1.bias,0.1)
        self.act1 = torch.nn.ReLU()
        self.f2 = torch.nn.Linear(200,self.n_actions)
        self.act2= torch.nn.Tanh()
        torch.nn.init.normal_(self.f2.weight,mean=0,std=0.1)
        torch.nn.init.constant_(self.f2.bias,0.1)
        self.act3 = torch.nn.Softplus()

    def forward(self,x):
        x=self.f1(x)
        x=self.act1(x)
        #new_opt =x.clone()
        x=self.f2(x)
        mu=self.act2(x)
        sigma=self.act3(x)
        return mu, sigma
        # self.dist_normal = torch.distributions.Normal(2*mu,sigma)
        # action = torch.clamp(self.dist_normal.sample(1),self.action_bound[0],self.action_bound[1])


class PolicyConstant_Gradient():
    def __init__(self,actor_net:PolicyNet,action_bound,lr=0.001,model_file=None):
        self.actor=actor_net
        self.learning_rate =lr
        self.action_bound =action_bound
        self.n_features = self.actor.n_features
        self.model_saved =model_file
        self.dist_normal= None
        self.optimizer=torch.optim.Adam(self.actor.parameters(),self.learning_rate)
        self.loss = None
    def choose_action(self,state):
        #state.astype(dtype=np.float32)
        state = torch.from_numpy(state).type(torch.FloatTensor)
        state=torch.squeeze(state,dim=0)
        mu,sigma = self.actor(state)
        self.dist_normal=torch.distributions.Normal(2*mu,sigma)
        action = torch.clamp(self.dist_normal.sample(),self.action_bound[0],self.action_bound[1])
        return action
    def update(self,obs_batch,action_batch,reward_batch):
        obs_batch = torch.from_numpy(obs_batch).type(torch.FloatTensor)
        action_batch=torch.from_numpy(action_batch).type(torch.FloatTensor)
        reward_batch = torch.from_numpy(reward_batch).type(torch.FloatTensor)
        mu,sigma = self.actor(obs_batch)
        self.dist_normal =torch.distributions.Normal(2*mu,sigma)
        log_prob = self.dist_normal.log_prob(action_batch)
        self.loss = - (log_prob*reward_batch+0.01*self.dist_normal.entropy()).mean()
        self.optimizer.zero_grad()
        self.loss.backward()
        self.optimizer.step()

def policy_ActorNet(env,policy,render_mode,test_iter):
    reward_sum = 0
    for i in range(test_iter):
        observation = env.reset()
        while True:
            if render_mode:
                env.render()
            state = np.reshape(observation,[1,3])
            action = policy.choose_action(state)
            observation_,reward,done,info = env.step(action)
            reward_sum+=reward
            if done:
                break
            observation = observation_

    return reward_sum
def policy_evaluator(env,agent:PolicyConstant_Gradient,training_iter,model_saved=None):
    reward_sum =0
    reward_sum_line =[]
    training_time=[]
    
    agent=agent
    
    env=env
    tmp =0
    for i in range(training_iter):
        tmp=i
        sampler=Sample(env,agent)
        train_obs,train_action,train_reward = sampler.sample_episodes(1)
        agent.update(train_obs,train_action,train_reward)
        
        if i==0:
            reward_sum = policy_ActorNet(env,agent,render_mode,1)
        else:
            reward_sum = 0.95*reward_sum + 0.05*policy_ActorNet(env,agent,render_mode,1)
        reward_sum_line.append(reward_sum)
        training_time.append(i)

        print("training episodes is {},trained reward sum is {}".format(i,reward_sum))

        if reward_sum>-500:
            break
    if model_saved:
        torch.save(agent,model_saved)
    else:
        torch.save(agent,'best_pg_pendulum_{}'.format(tmp+1))

    plt.plot(training_time,reward_sum_line,label="pg_pendulum_train")

    plt.xlabel("training iter")
    plt.ylabel("score")

    plt.legend()
    plt.show()


if __name__ =="__main__":

    env_name = "Pendulum-v0"
    env = gym.make(env_name)

    env.unwrapped

    env.seed(132)

    action_bound = np.array([-env.action_space.high,env.action_space.high]).reshape(2)
    policy_actor =PolicyNet(env)

    Agent = PolicyConstant_Gradient(policy_actor,action_bound)


    training_iter = 20000

    policy_evaluator(env,Agent,training_iter)

    reward_sum = policy_ActorNet(env,Agent,True,10)

    print("The Model tested Reward Sum is {}".format(reward_sum))














