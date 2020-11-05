#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2020 ACELAB

import os
import argparse
import csv
from datetime import date, timedelta, datetime

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


class BagToGpsCsv:
    def __init__(self, gps_topic, gps_time_topic, output_dir):
        ## set gps topic
        self.gps_topic = gps_topic
        self.gps_time_topic = gps_time_topic
        ## set directories and make folders if they are not exsist.
        self.root_dir = os.path.join(output_dir, "gps")
        createDir(self.root_dir)
        ## set gps-data properties
        self.seq = 0
        self.csv_file_path = os.path.join(self.root_dir, "gps.csv")
        self.csv_file = open(self.csv_file_path, "wb")
        self.gps_file = csv.writer(
            self.csv_file, delimiter=",", quotechar="|", quoting=csv.QUOTE_MINIMAL
        )
        ## set gps/utc time properties
        self.gps_weeks = 0
        self.gps_time_of_week_nsec = 0
        self.gps_time_offset = 0
        self.ros_time_ref = 0
        self.utc_year = 0
        self.utc_month = 0
        self.utc_day = 0
        self.utc_hour = 0
        self.utc_min = 0
        self.utc_sec = 0
        self.time_ref_set = False

    def processMsg(self, topic, msg, t, week, time_of_week, utc):
        if topic == self.gps_time_topic:
            self.setUtcTimeProperties(topic, msg, t)
        if topic == self.gps_topic:
            self.gps_file.writerow(
                [
                    self.seq,
                    t.to_sec(),
                    msg.latitude,
                    msg.longitude,
                    msg.altitude,
                    msg.status.status,
                    week,
                    time_of_week,
                    utc,
                ]
            )
            self.seq += 1

    def writeHeader(self):
        self.gps_file.writerow(
            [
                "sequence",
                "ros time [sec]",
                "latitude",
                "longitude",
                "altitude",
                "status",
                "week",
                "time of week [nsec]",
                "utc",
            ]
        )

    def __del__(self):
        self.csv_file.close()
        self.printSize()

    def printSize(self):
        print "BagToGpsCsvSummary:"
        print " - total message:", self.seq
        print " - total size:", os.path.getsize(self.csv_file_path), "bytes"

    def setUtcTimeProperties(self, topic, msg, t):
        if self.time_ref_set == True:
            "Warning! Time reference is already set!"
            return

        if topic == self.gps_time_topic:
            self.time_ref_set = True
            ## set utc time
            self.utc_year = msg.year
            self.utc_month = msg.month
            self.utc_day = msg.day
            self.utc_hour = msg.hour
            self.utc_min = msg.min
            self.utc_sec = msg.sec
            ## set gps week, time
            epoch = date(1980, 1, 6)
            today = date(int(msg.year), int(msg.month), int(msg.day))
            epochMonday = epoch - timedelta(epoch.weekday())
            todayMonday = today - timedelta(today.weekday())
            self.gps_weeks = (todayMonday - epochMonday).days // 7 - 1
            self.gps_time_of_week_nsec = msg.iTOW * 1000000
            self.ros_time_ref = t.to_nsec()
            return

    def getGpsTimeWeeks(self, rostime):
        return self.gps_weeks

    def getGpsWeekOfTimeNanosec(self, rostime):
        time_diff_nsec = rostime.to_nsec() - self.ros_time_ref
        return self.gps_time_of_week_nsec + time_diff_nsec

    def getUtctimeFromRosTime(self, rostime):
        time_diff_nsec = rostime.to_nsec() - self.ros_time_ref
        gps_nsec = self.gps_time_of_week_nsec + time_diff_nsec
        week_to_sec = 7 * 24 * 60 * 60
        to_nsec = 1000000000
        gps_timestamp = self.gps_weeks * week_to_sec + int(gps_nsec / to_nsec)
        gps_epoch_as_gps = datetime(1980, 1, 6)
        gps_time_as_gps = gps_epoch_as_gps + timedelta(seconds=gps_timestamp)
        gps_time_as_tai = gps_time_as_gps + timedelta(seconds=-8)  # constant offset
        tai_epoch_as_tai = datetime(1970, 1, 1, 0, 0, 10)
        tai_timestamp = (gps_time_as_tai - tai_epoch_as_tai).total_seconds()
        return datetime.utcfromtimestamp(tai_timestamp)


def main():
    parser = argparse.ArgumentParser(description="Extract gps from a ROS bag.")
    parser.add_argument("--bag_file", help="Input ROS bag.", type=str)
    parser.add_argument("--output_dir", help="Output directory.", type=str)
    parser.add_argument("--gps_topic", help="Gps topic.", type=str)

    args = parser.parse_args()

    print "Extract gps from {} on topic {} into {}".format(
        args.bag_file, args.gps_topic, args.output_dir
    )

    b2g = BagToGpsCsv(args.gps_topic, args.output_dir)
    b2g.writeHeader()

    ## open bag file
    try:
        bag = rosbag.Bag(args.bag_file, "r")
    except:
        print "file path is wrong, {}".format(args.bag_file)
        return

    for topic, msg, t in bag.read_messages(topics=[args.gps_topic]):
        b2g.processMsg(topic, msg, t)

    bag.close()
    del b2g

    return


if __name__ == "__main__":
    main()
