from DDPG import *
import matplotlib.pyplot as plt

if __name__ == '__main__':
    env = gym.make('CartPole-v0')
    ddpg = DDPG(4, 64, 1, 1, env, 1000, 1024, 5000, 0.001)
    steps = ddpg.learn()
    episodes = list(range(1, len(steps)+1))
    plt.plot(episodes, steps)
    plt.xlabel('episodes')
    plt.ylabel('steps')
    plt.title('steps-episodes')
    #plt.legend()
    plt.savefig('steps_episodes.svg')