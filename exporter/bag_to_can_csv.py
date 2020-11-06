#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2020 ACELAB

import os
import argparse
import csv

import rosbag
from sensor_msgs.msg import NavSatFix


def createDir(dir):
    output_dir = os.path.abspath(dir)
    try:
        if not (os.path.isdir(output_dir)):
            os.makedirs(os.path.join(output_dir))
    except OSError as e:
        if e.errno != errno.EEXIST:
            print ("Failed to create directory!!!!!")
            raise


class BagToCanCsv:
    def __init__(self, can_topic, output_dir):
        ## set directories and make folders if they are not exsist.
        self.root_dir = os.path.join(output_dir, "can")
        createDir(self.root_dir)
        ## set can-data properties
        self.seq = 0
        self.size = 0
        self.csv_file_path = os.path.join(self.root_dir, "can.csv")
        self.csv_file = open(self.csv_file_path, "wb")
        self.can_file = csv.writer(
            self.csv_file, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL
        )

    def processMsg(self, topic, msg, t, week, time_of_week, utc):
        self.can_file.writerow(
            [
                self.seq,
                t.to_sec(),
                msg.id,
                msg.is_rtr,
                msg.is_extended,
                msg.is_error,
                msg.dlc,
                msg.data.encode("hex"),
                week,
                time_of_week,
                utc[0],
                utc[1],
            ]
        )
        self.seq += 1

    def writeHeader(self):
        self.can_file.writerow(
            [
                "sequence",
                "ros time [sec]",
                "id",
                "is rtr",
                "is extended",
                "is error",
                "dlc",
                "data[hex]",
                "week",
                "time of week [nsec]",
                "utc [HHMMSS.SS]",
                "utc [DDMMYY]",
            ]
        )

    def __del__(self):
        self.csv_file.close()
        self.getSize()

    def getSize(self):
        print "BagToCanCsvSummary:"
        print " - total message:", self.seq
        print " - total size:", os.path.getsize(self.csv_file_path), "bytes"


def main():
    parser = argparse.ArgumentParser(description="Extract can from a ROS bag.")
    parser.add_argument("--bag_file", help="Input ROS bag.", type=str)
    parser.add_argument("--output_dir", help="Output directory.", type=str)
    parser.add_argument("--can_topic", help="can topic.", type=str)

    args = parser.parse_args()

    print "Extract can from {} on topic {} into {}".format(
        args.bag_file, args.can_topic, args.output_dir
    )

    b2c = BagToCanCsv(args.can_topic, args.output_dir)
    b2c.writeHeader()

    ## open bag file
    try:
        bag = rosbag.Bag(args.bag_file, "r")
    except:
        print "file path is wrong, {}".format(args.bag_file)
        return

    for topic, msg, t in bag.read_messages(topics=[args.can_topic]):
        b2c.processMsg(topic, msg, t)

    bag.close()
    del b2c

    return


if __name__ == "__main__":
    main()
