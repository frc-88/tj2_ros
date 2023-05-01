import torch
import torch.nn as nn


class BaseConv(nn.Module):
    pass


class RegularConv2d(BaseConv):
    def __init__(self, n_in, n_out):
        super(RegularConv2d, self).__init__()
        self.conv1 = nn.Conv2d(n_in, n_out, 3, 1, padding=1)

    def forward(self, x):
        conv1 = self.conv1(x)
        return conv1


class ParallelConv2d(nn.Module):
    def __init__(
        self, n_in, n_out, kernel_size, stride, padding, activation, classes_num
    ):
        super(ParallelConv2d, self).__init__()
        self.convs = nn.ModuleList(
            [
                nn.Conv2d(
                    n_in, n_out, kernel_size=kernel_size, stride=stride, padding=padding
                )
                for i in range(classes_num)
            ]
        )
        self.activation = activation

    def forward(self, xs):
        outs = []
        if type(xs) is torch.Tensor:
            for conv in self.convs:
                out = self.activation(conv(xs))
                outs.append(out)
        else:
            for x, conv in zip(xs, self.convs):
                out = self.activation(conv(x))
                outs.append(out)
        return outs


class ParallelFC(nn.Module):
    def __init__(self, n_in, n_out, activation):
        super(ParallelFC, self).__init__()
        self.fcs = nn.ModuleList([nn.Linear(n_in, n_out) for i in range(classes_num)])
        self.activation = activation

    def forward(self, xs):
        outs = []
        if type(xs) is torch.Tensor:
            for fc in self.fcs:
                out = self.activation(fc(xs))
                outs.append(out)
        else:
            for x, fc in zip(xs, self.fcs):
                out = self.activation(fc(x))
                outs.append(out)
        return outs
