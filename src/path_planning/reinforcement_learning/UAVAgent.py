import torch
from DQNModel import DQN
import numpy as np
import random
from collections import deque,namedtuple
import torch.nn.functional as F
class ReplayMemory:
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = deque(maxlen=capacity)

    def push(self, transition):
        self.memory.append(transition)

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)
    
class UAVAgent:
    
    def __init__(self):
        self.device = torch.device('cuda')
        
        self.state_size = 1
        self.action_size = 1
        self.epsilon = 0.001

        self.batch_size = 1
        self.memory = ReplayMemory(capacity=10000)

        self.lr = 1e-4
        self.Transition = namedtuple('Transition', ('state', 'action', 'reward', 'next_state','done'))
        self.criterion = torch.nn.MSELoss()
        self.target_update_interval = 5
        self.GAMMA = 0.99
        
        self.policy_net = DQN()
        # self.policy_net.load_state_dict(torch.load("/home/burakzdd/catkin_ws/src/deneme/scripts_one_axis/model/model.pth"))
        self.target_net = DQN()
        # self.target_net.load_state_dict(torch.load("/home/burakzdd/catkin_ws/src/deneme/scripts_one_axis/model/model.pth"))
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.optimizer = torch.optim.Adam(self.policy_net.parameters(), lr=self.lr)

                
    def findAction(self,state):
        # if random.uniform(0, 1) <= self.epsilon:
        #     return torch.tensor([[random.randint(1, 20)]])
        # else:
        state = torch.FloatTensor(state).unsqueeze(0)
        actions = self.policy_net(state).max(1).indices.view(-1, 1)
        return actions

    def remember(self,state,action,reward,next_state,done):
        self.memory.push(self.Transition(state,action,reward,next_state,done))
    
    def replay(self):
        
        if len(self.memory) < self.batch_size:
            return
        
        transitions = self.memory.sample(self.batch_size)
        batch = self.Transition(*zip(*transitions))
        
        state_batch = torch.stack([torch.tensor(state, dtype=torch.float32) for state in batch.state])
        next_state_batch = torch.stack([torch.tensor(next_state, dtype=torch.float32) for next_state in batch.next_state])
        action_batch = torch.cat(batch.action)
        reward_batch = torch.stack([torch.tensor(reward, dtype=torch.float32) for reward in batch.reward])
        done_batch = torch.stack([torch.tensor(state, dtype=torch.float32) for state in batch.done])
        
        Q_values = self.policy_net(state_batch).gather(1,action_batch)
        next_Q_values = self.target_net(next_state_batch).max(1).values
        expected_Q_values = (next_Q_values * self.GAMMA) + reward_batch
        # loss = self.criterion(Q_values, expected_Q_values.unsqueeze(1))
        loss = F.smooth_l1_loss(Q_values, expected_Q_values.unsqueeze(1))
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        # torch.nn.utils.clip_grad_value_(self.policy_net.parameters(), 100)
        # if self.current_step % self.target_update_interval == 0:
        self.update_target_model()
        
        torch.save(self.policy_net.state_dict(), '/home/burakzdd/catkin_ws/src/air_combat_simulation/scripts_one_axis/model/model.pth')   
        torch.save(self.target_net.state_dict(), '/home/burakzdd/catkin_ws/src/air_combat_simulation/scripts_one_axis/model/target_model.pth')
        return loss.item()
    
    def update_target_model(self):
        self.target_net.load_state_dict(self.policy_net.state_dict())