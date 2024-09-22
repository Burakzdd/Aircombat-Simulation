#!/usr/bin/env python3
import torch
import torch.nn as nn

class DQN(nn.Module):
    def __init__(self, input_size=1):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(1, 128)
        self.bn1 = nn.InstanceNorm1d(128)
        self.fc2 = nn.Linear(128, 256)
        
        self.bn2 = nn.InstanceNorm1d(256)
        self.fc3 = nn.Linear(256, 512)
        self.bn3 = nn.InstanceNorm1d(512)
        self.fc4 = nn.Linear(512, 256)
        self.bn4 = nn.InstanceNorm1d(256)
        self.fc_x = nn.Linear(256, 21)
        self.fc_y = nn.Linear(256, 21)
        self.fc_z = nn.Linear(256, 21)
        self.dropout = nn.Dropout(p=0.5)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = self.dropout(x)
        x = torch.relu(self.fc2(x))
        x = self.dropout(x)
        x = torch.relu(self.fc3(x))
        x = torch.relu(self.fc4(x))
        x_speed_x = torch.tanh(self.fc_x(x))

        return x_speed_x