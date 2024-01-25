#!/usr/bin/env python3

import os
os.environ["CUDA_VISIBLE_DEVICES"]= "0"

import torch
from torch import float32
# from polarnet import Polarnet
# import roslib; roslib.load_manifest('lidar_ciss_ros')

import rospy
# import geometry_msgs.msg
import std_msgs.msg

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import time

# from polarnet import Polarnet
from spvcnn import SPVCNN
import numpy as np
import yaml
import argparse
import struct
import math
import queue


def rotation_carla():
    cr = math.cos(math.radians(0))
    sr = math.sin(math.radians(0))
    cp = math.cos(math.radians(0))
    sp = math.sin(math.radians(0))
    cy = math.cos(math.radians(180))
    sy = math.sin(math.radians(180))
    return np.array([[cy*cp, -cy*sp*sr+sy*cr, -cy*sp*cr-sy*sr],[-sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],[sp, cp*sr, cp*cr]])

def callback(data):

    buffer = np.frombuffer(data.data, dtype=np.dtype([('x','f4'),('y','f4'),('z','f4'),('cos','f4')]))

    pc = np.array([buffer[:]['x'], buffer[:]['y'], buffer[:]['z'], buffer[:]['cos']])

    pc[:3,:]=(rotation_carla().T).dot(pc[:3,:])
    pc = pc.T

    start = time.time()
    result_label, result_xyz = model.infer(pc)
    result_xyz[:, 0] *= -1
    result_xyz[:, 1] *= -1
    print("model inference time: ", time.time()-start)
    print()

    CFG = yaml.safe_load(open("/home/ave/ros/catkin_ws_yoon/src/lidar_nciss_ros/src/semantic-kitti-carla.yaml", 'r'))
    color_dict = CFG["color_map"]
    pred_rgb_list = [color_dict[pred.item()] for pred in result_label]
    pred_rgb = np.stack(pred_rgb_list).astype(int)
    
    result = []
    for idx, xyz in enumerate(result_xyz):
        pt = xyz.tolist() + [0]
        rgb = struct.unpack('I', struct.pack('BBBB', pred_rgb[idx,0], pred_rgb[idx,1], pred_rgb[idx,2], 255))[0]
        pt[3] = rgb
        result.append(pt)
    

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1),
            ]

    header = Header()
    # header.stamp = rospy.Time.now()
    header.stamp = data.header.stamp
    header.frame_id = "ego_vehicle/lidar"


    msg = pc2.create_cloud(header, fields, result)

    pub.publish(msg)



if __name__ == '__main__':

    argparser = argparse.ArgumentParser(
    description=__doc__)
    argparser.add_argument(
        '--with_bldg',
        action='store_true',
        help='choose to use model trained with building or not (default: without building(false))')
    argparser.add_argument(
        '--ckpt',
        action='store_false',
        help='load the checkpoint'
    )

    args = argparser.parse_args()
    args.with_bldg = True

    args.num_classes = 12

    print('Creating lidar_ciss_node')
    rospy.init_node("lidar_ciss_node", anonymous=True)


    model = SPVCNN(args)

    if args.ckpt:
        
        #  carla checkpoint
        if args.with_bldg:
            print("After incremental learning (trained with building): Total (C) classes ")
            pretrained_checkpoint = "/home/ave/ros/catkin_ws_yoon/src/lidar_nciss_ros/src/pretrained/semantickitti/carla_pgt_building/last.ckpt"
        else:
            print("Before incremental learning (trained without building): Total (C-1) classes ")
            pretrained_checkpoint = "/home/ave/ros/catkin_ws_yoon/src/lidar_nciss_ros/src/pretrained/semantickitti/carla_2DPASS_semkitti_ysh_building/last.ckpt"

        model_dict = model.state_dict()
        pretrained_dict = torch.load(pretrained_checkpoint)['state_dict']
        update_dict = {}
        for k, v in pretrained_dict.items():
            if 'model_3d' in k:
                update_dict[k.replace('model_3d.','')] = v
            elif 'fusion.classifier' in k:
                update_dict[k.replace('fusion.','')] = v
            else:
                continue

        model_dict.update(update_dict)
        model.load_state_dict(model_dict, strict=False)
        # model.load_state_dict(model_dict, strict=True)

    else:
        checkpoint = None


    rospy.Subscriber('/carla/ego_vehicle/lidar', PointCloud2, callback, buff_size = 65536*16, queue_size=1)
    pub = rospy.Publisher("lidar_ciss/lidar", PointCloud2, queue_size=1)
    


    rospy.spin()


