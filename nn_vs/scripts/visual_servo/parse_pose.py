import os
import numpy as np


def quaternion_multiply(q1, q2):
    """
    q1 * q2 =
    (w1*w2 - x1*x2 - y1*y2 - z1*z2)   + (w1*x2 + x1*w2 + y1*z2 - z1*y2) i +
    (w1*y2 - x1*z2 + y1*w2 + z1*x2) j + (w1*z2 + x1*y2 - y1*x2 + z1*w2) k
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return [x, y, z, w]


def quaternion_normal(q):
    x, y, z, w = q
    mod = pow((pow(x, 2) + pow(y, 2) + pow(z, 2) + pow(w, 2)), 1 / 2)
    return [x / mod, y / mod, z / mod, w / mod]


def quaternion_inv(q):
    x, y, z, w = quaternion_normal(q)
    return [-x, -y, -z, w]


def parse_pose(path):
    """Parses the pose file and return position and quaternion """
    if not path.endswith('.txt'):
        path += '.txt'

    file = open(path, 'r')  # 读取文件
    pose = []  # 创建列表, 列表中的元素为列表
    for line in file.readlines():  # 逐行读取
        line = line.strip()  # 消除行头尾的空白符(空格, 回车等)
        if not line:  # 如果遇到空行或者注释行, 则跳过
            continue
        if line.startswith('#'):  # 遇到模块的起始, 在列表后添加新的列表
            pose.append([])
        else:
            key, value = line.split('=')  # 根据参数值为字典赋值, 注意要去除空白符
            pose[-1].append(float(value.strip()))
    return pose


# position : x,y,z
# quaternion: x,y,z,w
# rpy:r,p,y

def parse_delta_pose(path1, path2):
    p1, q1, _ = parse_pose(path1)
    if q1[0] < 0: q1 = [x * (-1) for x in q1]
    p2, q2, _ = parse_pose(path2)
    if q2[0] < 0: q2 = [x * (-1) for x in q2]
    delta_p = [(n - m) for m, n in zip(p1, p2)]
    q1_ = quaternion_inv(q1)
    delta_q = quaternion_normal(quaternion_multiply(q2, q1_))
    # 0.99 / 100 -> 0.009
    delta_q[3] /= 100
    # print(q1, q2)
    # print(delta_q)
    # print(quaternion_multiply(delta_q, q1))
    # print(quaternion_normal(delta_q))
    # print(quaternion_multiply(delta_q, q1))
    return delta_p + delta_q  # [x,y,z, x,y,z,w]


# if __name__ == '__main__':
#     path1 = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/poses/1.txt'
#     path2 = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/poses/2.txt'
#
#     # pose = parse_pose(path1)
#     # print(pose)
#     delat_pose = parse_delta_pose(path1, path2)
#     print(delat_pose)


