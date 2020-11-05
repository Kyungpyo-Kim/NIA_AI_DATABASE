#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2020 ACELAB

"""Extract data from a rosbag.
"""

import os
import argparse

import rosbag
from tqdm import tqdm

from bag_to_image import BagToImage
from bag_to_gps_csv import BagToGpsCsv
from bag_to_can_csv import BagToCanCsv

def main():
  """Extract a folder of data from a rosbag.
  """
  parser = argparse.ArgumentParser(
    description="Extract data from a ROS bag.")
  parser.add_argument("--bag_file", help="Input ROS bag.", type=str, required=True)
  parser.add_argument("--output_dir", help="Output directory.", type=str, default='./out')
  parser.add_argument("--image_topic", help="Image topic.", type=str, default='/pylon_camera_node/image_raw')
  parser.add_argument("--gps_topic", help="Gps topic.", type=str, default='/ublox_gps/fix')
  parser.add_argument("--gps_time_topic", help="Gps time topic.", type=str, default='/ublox_gps/navpvt')
  parser.add_argument("--can_topic", help="Can topic.", type=str, default='/can_tx')

  args = parser.parse_args()
  bag_file_path = os.path.realpath(args.bag_file)

  print "\nParsed Arguments:\n"
  print " - bag_file:", args.bag_file
  print "   :", bag_file_path
  print " - output_dir:", args.output_dir
  print " - image_topic:", args.image_topic
  print " - gps_topic:", args.gps_topic
  print " - gps_time_topic:", args.gps_time_topic
  print " - can_topic:", args.can_topic

  ## open bag file or folder
  bag_file_list = []
  if os.path.isfile(bag_file_path) and bag_file_path[-4:] == ".bag":
    bag_file_list.append(bag_file_path)
  else:
    bag_file_list = [os.path.join(bag_file_path, f) for f in os.listdir(bag_file_path) if f[-4:] == ".bag"]

  bag_file_list.sort()
  print "\nTotal number of bag files: {}\n".format(len(bag_file_list))
  for f in bag_file_list:
    print f

  for i, bag_file in enumerate(bag_file_list):

    out_path = os.path.join(args.output_dir, bag_file.split('/')[-1][:-4] )
    b2i = BagToImage(args.image_topic, out_path)
    b2i.writeHeader()    
    b2g = BagToGpsCsv(args.gps_topic, args.gps_time_topic, out_path)
    b2g.writeHeader()    
    b2c = BagToCanCsv(args.can_topic, out_path)
    b2c.writeHeader()
    
    try: 
      bag = rosbag.Bag(bag_file, "r")
    except:
      print "file path is wrong, {}".format(bag_file)
      continue

    ## set gps utc time offset
    for topic, msg, t in bag.read_messages(topics=[args.gps_time_topic]):
      if topic == args.gps_time_topic:
        b2g.setUtcTimeProperties(topic, msg, t)
        print "\nSet gps utc time\n"
        print " - Gps time week:", b2g.getGpsTimeWeeks(t)
        print " - Gps week of time [nsec]:", b2g.getGpsWeekOfTimeNanosec(t)
        print " - UTC time:", b2g.getUtctimeFromRosTime(t)
        break
    if b2g.time_ref_set == False:
      print "\n!! Warning, No GPS Time Reference !!\n"

    ## get data
    msg_num = bag.get_message_count(args.image_topic)
    msg_num += bag.get_message_count(args.gps_topic)
    msg_num += bag.get_message_count(args.can_topic)

    print "\nStart data parsing: {}\n".format(bag_file)
    for topic, msg, t in tqdm(bag.read_messages(topics=[args.image_topic, args.gps_topic, args.can_topic]), total=msg_num):
      if topic == args.image_topic:
        b2i.processMsg(topic, msg, t, b2g.getGpsTimeWeeks(t), 
          b2g.getGpsWeekOfTimeNanosec(t), b2g.getUtctimeFromRosTime(t))
      if topic == args.gps_topic or topic == args.gps_time_topic:
        b2g.processMsg(topic, msg, t, b2g.getGpsTimeWeeks(t), 
          b2g.getGpsWeekOfTimeNanosec(t), b2g.getUtctimeFromRosTime(t))
      if topic == args.can_topic:
        b2c.processMsg(topic, msg, t, b2g.getGpsTimeWeeks(t), 
          b2g.getGpsWeekOfTimeNanosec(t), b2g.getUtctimeFromRosTime(t))

    print "\nData parsing finished\n"

    bag.close()
    del b2i, b2c, b2g

  return

if __name__ == '__main__':
  main()