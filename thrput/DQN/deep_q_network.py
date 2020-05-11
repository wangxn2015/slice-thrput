import os
import torch as T
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np

class DeepQNetwork(nn.Module):
    def __init__(self, lr, n_actions, name, input_dims, chkpt_dir):
        super(DeepQNetwork, self).__init__()
        self.checkpoint_dir = chkpt_dir
        self.checkpoint_file = os.path.join(self.checkpoint_dir, name)

        # self.conv1 = nn.Conv2d(input_dims[0], 32, 8, stride=4)
        # self.conv2 = nn.Conv2d(32, 64, 4, stride=2)
        # self.conv3 = nn.Conv2d(64, 64, 3, stride=1)

        # fc_input_dims = self.calculate_conv_output_dims(input_dims)
        
        self.bfc0 = nn.Linear(4, 256)        
        self.fc0 = nn.Linear(256, 512)
        self.fc1 = nn.Linear(512, 512)
        # self.fc1 = nn.Linear(fc_input_dims, 512)
        self.fc2 = nn.Linear(512, n_actions)

        self.optimizer = optim.RMSprop(self.parameters(), lr=lr)

        self.loss = nn.MSELoss()

        print("cuda available: ",T.cuda.is_available())
        self.device = T.device('cuda:0' if T.cuda.is_available() else 'cpu')
        # todo change it back to cuda
        # self.device = T.device('cpu')

        self.to(self.device)

    # def calculate_conv_output_dims(self, input_dims):
    #     state = T.zeros(1, *input_dims)
    #     dims = self.conv1(state)
    #     dims = self.conv2(dims)
    #     dims = self.conv3(dims)
    #     return int(np.prod(dims.size()))

    def forward(self, state):
        # conv1 = F.relu(self.conv1(state))
        # conv2 = F.relu(self.conv2(conv1))
        # conv3 = F.relu(self.conv3(conv2))
        # conv3 shape is BS x n_filters x H x W
        # conv_state = conv3.view(conv3.size()[0], -1)
        # conv_state shape is BS x (n_filters * H * W)
        
        bflat0 = F.relu(self.bfc0(state))
        flat0 = F.relu(self.fc0(bflat0))
        flat1 = F.relu(self.fc1(flat0))
        actions = self.fc2(flat1)

        return actions

    def save_checkpoint(self):
        print('... saving checkpoint ...')
        T.save(self.state_dict(), self.checkpoint_file)

    def load_checkpoint(self):
        print('... loading checkpoint ...')
        self.load_state_dict(T.load(self.checkpoint_file))
