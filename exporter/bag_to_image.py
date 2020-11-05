#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2020 ACELAB

import os
import argparse
import csv

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def createDir(dir):
    output_dir = os.path.abspath(dir)
    try:
        if not (os.path.isdir(output_dir)):
            os.makedirs(os.path.join(output_dir))
    except OSError as e:
        if e.errno != errno.EEXIST:
            print ("Failed to create directory!!!!!")
            raise


class BagToImage:
    def __init__(self, image_topic, output_dir):
        ## set directories and make folders if they are not exsist.
        self.root_dir = os.path.join(output_dir, "image")
        self.images_dir = os.path.join(self.root_dir, "images")
        createDir(self.root_dir)
        createDir(self.images_dir)
        ## set image message properties
        self.image_topic = image_topic
        self.bridge = CvBridge()
        ## set meta-data properties
        self.seq = 0
        self.size = 0
        self.csv_file_path = os.path.join(self.root_dir, "meta.csv")
        self.csv_file = open(self.csv_file_path, "wb")
        self.meta_file = csv.writer(
            self.csv_file, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL
        )

    def processMsg(self, topic, msg, t, week, time_of_week, utc):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        image_path = os.path.join(self.images_dir, "frame%06i.png" % self.seq)
        cv2.imwrite(image_path, cv_img)
        self.meta_file.writerow(
            [self.seq, t.to_sec(), image_path, week, time_of_week, utc]
        )
        self.seq += 1
        self.size += os.path.getsize(image_path)

    def writeHeader(self):
        self.meta_file.writerow(
            [
                "sequence",
                "ros time [sec]",
                "image path (relative)",
                "week",
                "time of week [nsec]",
                "utc",
            ]
        )

    def __del__(self):
        self.csv_file.close()
        self.getSize()

    def getSize(self):
        print "BagToImage:"
        print " - total message:", self.seq
        self.size += os.path.getsize(self.csv_file_path)
        print " - total size:", self.size / 1024, "Mbytes"


def main():
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("--bag_file", help="Input ROS bag.", type=str)
    parser.add_argument("--output_dir", help="Output directory.", type=str)
    parser.add_argument("--image_topic", help="Image topic.", type=str)

    args = parser.parse_args()

    print "Extract images from {} on topic {} into {}".format(
        args.bag_file, args.image_topic, args.output_dir
    )
    bti = BagToImage(args.image_topic, args.output_dir)
    bti.writeHeader()

    ## open bag file
    try:
        bag = rosbag.Bag(args.bag_file, "r")
    except:
        print "file path is wrong, {}".format(args.bag_file)
        return

    bridge = CvBridge()
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        bti.processMsg(topic, msg, t)

    bag.close()
    del bti

    return


if __name__ == "__main__":
    main()
