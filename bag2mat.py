'''
/* Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang@umich.edu)
 * WEBSITE: https://www.brucerobot.com/
 */


[HOW TO USE]
To run this file, please change the path in main function and change topics names if
you have other topics you would like to extract.

If you have all patches extracted, please set extracted = 1, then it will only convert
the bagfiles to mat-files.

If you have not extracted patches from /velodyne_points, please set extracted = 0,
and specify a cuboid in the readPointsAndSave function.

If you have all the extracted files in a folder, please set load_folder = 1, it will
go ahead convert all the files to mat-files; please also remember to set the topic
names.

Please run this by:
    python bag2mat.py [bagfile.bag] [event-name]
    [bagfile.bag]: name of the bagfile
    [event-name]: whatever you would like to name this
'''




import rosbag
import scipy.io as sio
import sensor_msgs.point_cloud2 as pc2

import numpy as np
import os.path
import sys
import datetime

def walk_dir(path, key):
    list_of_files = []
    for (dirpath, dirnames, filenames) in os.walk(path):
        for filename in filenames:
            if filename.endswith(key): 
                list_of_files.extend([os.sep.join([dirpath, filename])])

    return list_of_files

def readPointsAndSave(bag, topic, time, if_filter):
    max_point = 0
    scan_count = 0

    # find out how many points and
    # how many scan in this bagfile
    for topic, cloud, t in bag.read_messages(topics=topic):
        if cloud is not None:
            scan_count += 1
            num_point = len(list(pc2.read_points(cloud, skip_nans=True)))
            if  num_point > max_point:
                max_point = num_point

    all_points = 0
    scan = 0
    point_cloud = np.zeros(shape=(scan_count, max_point, 5))
    for topic, cloud, t in bag.read_messages(topics=topic):
        if cloud is not None:
            if len(cloud.data) == 0: # if this scan is empty
                continue

            point_count = 0
            points = np.zeros(shape=(max_point, 5), dtype=object)

            for point in pc2.read_points(cloud, skip_nans=True):
                if point is None:
                    continue;

                ### XXX: MODIFY the cuboid
                if if_filter == 1: # extract 3 payloads
                    ## tag 1 
                    # filter by x
                    if ((point[0] < 4) or (point[0] > 4.2)): 
                        continue

                    # filter by y
                    if ((point[1] > 0.1) or (point[1] < -0.25)):
                        continue

                    # filter by z
                    if (point[2] > -0.25) or (point[2] < -0.55):
                        continue

                    ## tag 2
                    # if ((point[0] < 10.5) or (point[0] > 11.6)): 
                    #     continue

                    # if ((point[1] > 2.8) or (point[1] < 1.9)):
                    #     continue

                    # if (point[2] > 0.3) or (point[2] < -0.70):
                    #     continue

                    ## tag 3
                    # if ((point[0] < 16) or (point[0] > 16.7)): 
                    #     continue

                    # if ((point[1] > -2.6) or (point[1] < -4.2)):
                    #     continue

                    # if (point[2] > 0.35) or (point[2] < -1.1):
                    #     continue

                point = tuple((point[0], point[1], point[2], point[3], point[4]))
                points[point_count, :] = point # in X Y Z I R format
                point_count += 1
                all_points += 1
            point_cloud[scan, :, :] = points
            scan += 1

    name = topic[topic.rfind('/')+1:] + "-"
    if  scan_count == 0:
        print 'no scans in ', name
    else:
        print '===== Statistic: ====='
        print 'finished topic: ', topic
        print 'number of scans: ', scan_count
        print 'max point: ', max_point
        print 'ave points: ', float(all_points/scan_count)
        print 'size: ', point_cloud.shape
        sio.savemat(name + time + '.mat', {'point_cloud': point_cloud})
        print 'saved: ', str(name + time + '.mat')

def convertToMatFile(path_to_bagfile, topics, name, extracted):
    now = datetime.datetime.now()
    time = now.strftime("%Y-%m-%d-%H-%M")
    event_name = name + "--" + time
    bag = rosbag.Bag(path_to_bagfile)

    for topic in topics:
        if topic == "/velodyne_points":
            if extracted:
                # show all points
                readPointsAndSave(bag, topic, event_name, 0) 
            else:
                # extract targets 
                readPointsAndSave(bag, topic, event_name, 1)
        else:
            # conver to mat-file from other topics
            # assume patches have been extracted out
            readPointsAndSave(bag, topic, event_name, 0) 


    bag.close()
    print 'finished bagfile: ', bagfile

if __name__ == '__main__':
    # XXX: MODIFY
    load_folder = 0 #  conver all files in a folder
    extracted = 0 # if extraction needed
    path = "/PATH_TO_BAGFILES/"
    read_topic = "/LiDARTag"

    if load_folder:
        if len(sys.argv)==1:
        elif len(sys.argv)==2:
            path = sys.argv[1]
        elif len(sys.argv)==3:
            path = sys.argv[1]
            read_topic = sys.argv[2]
        else:
            print("wrong number of arguments")
            sys.exit();

        print 'Loading rosbag from: ', path
        print 'Topic is: ', read_topic

        ros_bags = walk_dir(path, key = '.bag')
        num_bags = len(ros_bags)

        # total, training, testing, bad_scan (too many points), original scan_num
        for bag in sorted(ros_bags):
            bag_id = int(str(bag[bag.rfind('.')-2]) + str(bag[bag.rfind('.')-1]))
            
            # assume patches have been extracted
            convertToMatFile(bag_id, bag, read_topic, 1) 
    else:
        if len(sys.argv)==1:
            bag_name = "bagfile.bag" 
            event_name = "event-name"
            read_topic1 = "/LiDARTag"
            read_topic2 = "/LiDARTag/DetectedPC"
            read_topic3 = "/velodyne_points"
        elif len(sys.argv)==2:
            bag_name = sys.argv[1]
            event_name = "event-name"
            read_topic1 = "/LiDARTag"
            read_topic2 = "/LiDARTag/DetectedPC"
            read_topic3 = "/velodyne_points"
        elif len(sys.argv)==3:
            bag_name = sys.argv[1]
            event_name = sys.argv[2]
            read_topic1 = "/LiDARTag"
            read_topic2 = "/LiDARTag/DetectedPC"
            read_topic3 = "/velodyne_points"
        elif len(sys.argv)==4:
            bag_name = sys.argv[1]
            event_name = sys.argv[2]
            read_topic1 = sys.argv[3]
            read_topic2 = "/LiDARTag/DetectedPC"
            read_topic3 = "/velodyne_points"
        else:
            print("wrong number of arguments")
            sys.exit();

        topics = [read_topic1, read_topic2, read_topic3]
        topics = [read_topic3]
        bagfile = path + bag_name 
        print 'Loading rosbag: ', bagfile
        print 'Topics are: ', topics

        convertToMatFile(bagfile, topics, event_name, extracted)

    print 'finished loading bagfiles' 
