#!~/anaconda3/bin/env	python3

import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
import torch.nn.functional as F
from torch.autograd import Variable
import os
from os import listdir, getcwd
from os.path import join
from PIL import Image
import copy
import random


class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.feature_extraction = nn.Sequential(
            # (128 + 0 - 5)//2 + 1 = 62  # N,8,62,62
            nn.Conv2d(in_channels=3, out_channels=8, kernel_size=5, stride=2, padding=0, bias=False),
            nn.LeakyReLU(inplace=True),
            # (62+0-5)/2+1=29
            nn.Conv2d(in_channels=8, out_channels=8, kernel_size=5, stride=2, padding=0, bias=False),
            nn.LeakyReLU(inplace=True),
            # (29+0-5)/2+1=13
            nn.Conv2d(in_channels=8, out_channels=16, kernel_size=5, stride=2, padding=0, bias=False),
            nn.LeakyReLU(inplace=True),
            # (13+0-5)/2+1=5
            nn.Conv2d(in_channels=16, out_channels=16, kernel_size=5, stride=2, padding=0, bias=False),
            nn.LeakyReLU(inplace=True),

            nn.AvgPool2d(kernel_size=5),

            nn.Conv2d(in_channels=16, out_channels=16, kernel_size=1, stride=1, padding=0, bias=False),
            nn.LeakyReLU(inplace=True),
            nn.Conv2d(in_channels=16, out_channels=3, kernel_size=1, stride=1, padding=0, bias=False),
            #             nn.ReLU(inplace=True),
        )

        self.sigmoid = nn.Sigmoid()
        self.tanh = nn.Tanh()

    def forward(self, x):
        x = self.feature_extraction(x)
        m, n = x.split([2, 1], dim=1)
        m = self.sigmoid(m)
        n = self.tanh(n)
        x = torch.cat((m, n), dim=1)
        return x


# Define criteria
MSE = nn.MSELoss()
def loss_fn(pred, target):
    ft = torch.cuda.FloatTensor if pred[0].is_cuda else torch.Tensor
    lpos, lcos = ft([0]), ft([0])
    # print(len(pred))
    if len(pred) == 1:
        pred = pred.squeeze()
        target = target.squeeze()
        lpos += torch.mean(torch.pow((pred[0] - target[0]), 2))
        if target[0] == 1:
            # lcos += MSE(p[1:], t[1:])
            lcos += torch.mean(torch.pow((pred[1:] - target[1:]), 2))
    else:
        pred = pred.squeeze()
        # print(target.shape)
        # print(pred.shape)
        for p, t in zip(pred, target):
            # lpos += MSE(p[0], t[0])
            lpos += torch.mean(torch.pow((p[0] - t[0]), 2))
            if t[0] == 1:
                # lcos += MSE(p[1:], t[1:])
                lcos += torch.mean(torch.pow((p[1:] - t[1:]), 2))
    loss = lpos + lcos
    return loss, lpos
