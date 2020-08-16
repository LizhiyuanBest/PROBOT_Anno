#!~/anaconda3/bin/env	python3
import torch
import torch.nn as nn
import torchvision


class Net(nn.Module):
    def __init__(self, alexnet_model):
        super(Net, self).__init__()
        self.feature_extraction = nn.Sequential(*list(alexnet_model.children())[:-2])
        for p in self.parameters():
            p.requires_grad = False
        self.reduce_channels_flatten = nn.Sequential(
            # reduce the channels  # 256 --> 96  # [N, 96, 19, 14]
            nn.Conv2d(in_channels=256, out_channels=96, kernel_size=1, stride=1, padding=0, bias=False),
            nn.ReLU(inplace=True),

            # Flatten # [N, 96*19*14]
            nn.Flatten()
        )

        self.classifier = nn.Sequential(
            nn.Dropout(p=0.5),
            nn.Linear(in_features=96 * 19 * 14 * 2, out_features=4096),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.5),
            nn.Linear(in_features=4096, out_features=4096),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.5),
            nn.Linear(in_features=4096, out_features=4096),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.5),
            nn.Linear(in_features=4096, out_features=1024),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.5),
            nn.Linear(in_features=1024, out_features=1024),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.5),
            nn.Linear(in_features=1024, out_features=1024),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.5),
            nn.Linear(in_features=1024, out_features=1024),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.5),
            nn.Linear(in_features=1024, out_features=7),
        )

    def forward(self, x1, x2):
        x1 = self.feature_extraction(x1)
        x1 = self.reduce_channels_flatten(x1)
        x2 = self.feature_extraction(x2)
        x2 = self.reduce_channels_flatten(x2)
        # x1 = x1.view(x1.size(0), -1)
        # x2 = x2.view(x2.size(0), -1)
        # Concatenate
        x = torch.cat((x1, x2), dim=1)
        x = self.classifier(x)
        return x


# alexnet_model = torchvision.models.alexnet(pretrained=True)
# model = Net(alexnet_model)
# x1 = torch.rand(2, 3, 480, 640)
# x2 = torch.rand(2, 3, 480, 640)
# y = model(x1, x2)
# print(y.shape)
