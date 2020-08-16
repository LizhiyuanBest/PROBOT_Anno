#!~/anaconda3/bin/env	python3

import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.optim as optim
import torchvision
from parse_pose import *
from model import Net
import os
from os import listdir, getcwd
from os.path import join
from PIL import Image
import copy
import random
import math

ROOT_Path = '/home/zyli/visual_servo'
Image_Train_Path = '/home/zyli/visual_servo/data/images/train'
Image_Val_Path = '/home/zyli/visual_servo/data/images/val'
Pose_Path = '/home/zyli/visual_servo/data/poses'
Weights_Path = '/home/zyli/visual_servo/weights'

img_mean = [0.56686793, 0.5620438, 0.56201945]
img_std = [0.05689577, 0.08067945, 0.08079015]
p_mean = [0.30000355, -0.0450017, 0.24000011]
p_std = [0.003162, 0.00316268, 0.00316231]
q_mean = [9.97840382e-01, -1.10953997e-05, -1.07665062e-04, 2.31655956e-04]
q_std = [0.00132296, 0.05290351, 0.02750256, 0.02752423]


# img_mean:[0.56686793 0.5620438  0.56201945]
# img_std:[0.05689577 0.08067945 0.08079015]
# p_mean:[ 0.30000355 -0.0450017   0.24000011]
# p_std:[0.003162   0.00316268 0.00316231]
# q_mean:[ 9.97840382e-01 -1.10953997e-05 -1.07665062e-04  2.31655956e-04]
# q_std:[0.00132296 0.05290351 0.02750256 0.02752423]


def normal(pred_r):
    # print(pred_r.shape)
    x, y, z, w = pred_r.split([1, 1, 1, 1], dim=1)
    # print(x.shape,y.shape,z.shape,w.shape)
    mod = pow((pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(w, 2)), 1 / 2)
    # print(mod.shape)
    pred_r = torch.cat([x / mod, y / mod, z / mod, w / mod], 1)
    return pred_r


loss_weight = 0.99


def loss_fn(pred, target):
    # decomposed into translation and rotation
    target_t, target_r = target.split([3, 4], dim=1)
    pred_t, pred_r = pred.split([3, 4], dim=1)

    # pred_r = normal(pred_r)
    # print(pred_r.shape)
    loss_t = torch.sqrt(torch.mean(torch.pow((target_t - pred_t), 2)))
    loss_r = torch.sqrt(torch.mean(torch.pow((target_r - pred_r), 2)))
    # print(target_r, pred_r)
    # print(loss_t, loss_r)
    return loss_t * loss_weight + loss_r * (1 - loss_weight)


def calc_labels(img1_files, img2_files, batch_size, Image_path):
    img1s = []
    img2s = []
    delta_poses = []
    for img1_file, img2_file in zip(img1_files, img2_files):
        pose1_file = img1_file.replace('jpg', 'txt')
        img1 = Image.open(join(Image_path, img1_file))
        img1 = np.array(img1, dtype=np.float)
        img1 = torch.FloatTensor(img1).permute(2, 0, 1).unsqueeze(0)
        img1s.append(img1)
        pose2_file = img2_file.replace('jpg', 'txt')
        img2 = Image.open(join(Image_path, img2_file))
        img2 = np.array(img2, dtype=np.float)
        img2 = torch.FloatTensor(img2).permute(2, 0, 1).unsqueeze(0)
        img2s.append(img2)

        fp1 = join(Pose_Path, pose1_file)
        fp2 = join(Pose_Path, pose2_file)
        delta_pose = parse_delta_pose(fp1, fp2)
        delta_pose = torch.FloatTensor(np.array(delta_pose, dtype=np.float)).unsqueeze(0)
        delta_poses.append(delta_pose)
        # print(delta_pose)

    img1s = torch.cat(img1s, 0)
    img2s = torch.cat(img2s, 0)
    delta_poses = torch.cat(delta_poses, 0)

    return img1s, img2s, delta_poses


def quaternion_to_rpy(q):
    x, y, z, w = q
    rpy_r = math.atan2(2.0 * (x * w + y * z), 1.0 - 2.0 * (x * x + y * y))
    rpy_p = math.asin(2.0 * (w * y - z * x))
    rpy_y = math.atan2(2.0 * (z * w + y * x), 1.0 - 2.0 * (z * z + y * y))
    return [rpy_r, rpy_p, rpy_y]


def calc_accuracys(img1_files, img2_files, delta_poses, batch_size):
    delta_poses = delta_poses.detach().numpy()
    accs = []
    for img1_file, img2_file, delta_pose in zip(img1_files, img2_files, delta_poses):
        pose1_file = img1_file.replace('jpg', 'txt')
        fp1 = join(Pose_Path, pose1_file)
        p1, q1, _ = parse_pose(fp1)
        pose2_file = img2_file.replace('jpg', 'txt')
        fp2 = join(Pose_Path, pose2_file)
        p2, q2, _ = parse_pose(fp2)
        pred_p2 = [(m + n) for m, n in zip(p1, delta_pose[:3])]
        # pred_q2 = quaternion_normal(delta_pose[3:])
        delta_pose[6] *= 100
        pred_q2 = quaternion_multiply(delta_pose[3:], q1)
        pred_q2 = quaternion_normal(pred_q2)
        rpy2 = quaternion_to_rpy(q2)
        pred_rpy2 = quaternion_to_rpy(pred_q2)
        # print(delta_pose)
        # print(q2, pred_q2)
        # print(rpy2, pred_rpy2)
        acc = np.array(p2 + rpy2) - np.array(pred_p2 + pred_rpy2)
        # acc = np.array(p2 + q2) - np.array(pred_p2 + pred_q2)

        accs.append(acc)
    accs = np.array(accs)
    # print(accs.shape)
    accuracy = np.mean(accs, axis=0, keepdims=True)

    return accuracy

