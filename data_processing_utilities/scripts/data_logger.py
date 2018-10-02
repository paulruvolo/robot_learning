#!/usr/bin/env python

import rospy
import Queue as queue
from sensor_msgs.msg import LaserScan, Image
from neato_node.msg import Bump, Accel
from cv_bridge import CvBridge
import cv2
import rospkg
import errno
import os
import csv


class DataLogger(object):
    def __init__(self):
        self.last_ranges = None
        self.last_bump = None
        self.last_accel = None
        self.lbutton_down_registered = False
        self.last_x, self.last_y = -1, -1
        self.q = queue.Queue()
        self.sensor_latency_tolerance = rospy.Duration(0.3)

        rospy.init_node('data_logger')
        r = rospkg.RosPack()
        self.data_dir = rospy.get_param('~data_dir', 'mydataset')
        self.data_save_path = r.get_path('data_processing_utilities') + \
            '/data/' + self.data_dir
        try:
            os.mkdir(self.data_save_path)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
        rospy.loginfo("data save directory " + self.data_save_path)
        rospy.Subscriber('extrapolated_scan', LaserScan, self.process_scan)
        rospy.Subscriber('camera/image_raw', Image, self.process_image)
        rospy.Subscriber('bump', Bump, self.process_bump)
        rospy.Subscriber('accel', Accel, self.process_accel)
        self.b = CvBridge()
        cv2.namedWindow('camera image')
        cv2.setMouseCallback('camera image', self.process_mouse_event)

    def process_scan(self, msg):
        self.last_ranges = (msg.header.stamp, msg.ranges)

    def process_mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # this flag gets consumed elsewhere
            self.lbutton_down_registered = True
        self.last_x, self.last_y = x, y

    def process_bump(self, msg):
        self.last_bump = (rospy.Time.now(),
                          (msg.leftFront,
                           msg.leftSide,
                           msg.rightFront,
                           msg.rightSide))

    def process_accel(self, msg):
        self.last_accel = (rospy.Time.now(),
                           (msg.accelXInG, msg.accelYInG, msg.accelZInG))

    def process_image(self, m):
        self.q.put((m.header.stamp,
                    self.last_x,
                    self.last_y,
                    self.lbutton_down_registered,
                    self.b.imgmsg_to_cv2(m, desired_encoding="passthrough")))
        self.lbutton_down_registered = False

    def run(self):
        with open(self.data_save_path + "/metadata.csv", "w") as csv_file:
            writer = csv.writer(csv_file)
            image_count = 0
            writer.writerows(['stamp',
                              'mouse_x',
                              'mouse_y',
                              'lbutton_down',
                              'key'] +
                             ['ranges_' + str(i) for i in range(361)] +
                             ['bump_leftFront',
                              'bump_leftSide',
                              'bump_rightFront',
                              'bump_rightSide'] +
                             ['accelXInG', 'accelYInG', 'accelZInG'])
            while not rospy.is_shutdown():
                stamp, x, y, lbutton_down, image = self.q.get(timeout=10)

                # TODO: double check that this is actual 361 (we are using
                # extrapolated scan so I don't know if it is or isn't)
                scan = [float('Inf')]*361
                if (self.last_ranges and abs(stamp - self.last_ranges[0]) <
                        self.sensor_latency_tolerance):
                    scan = self.last_ranges[1]

                bump = [float('Inf')]*4
                if (self.last_bump and abs(stamp - self.last_bump[0]) <
                        self.sensor_latency_tolerance):
                    bump = self.last_bump[1]

                accel = [float('Inf')]*3
                if (self.last_accel and abs(stamp - self.last_accel[0]) <
                        self.sensor_latency_tolerance):
                    accel = self.last_accel[1]

                cv2.imshow("camera image", image)
                key = cv2.waitKey(5) & 0xFF

                filename = "%010d.jpg" % (image_count,)
                print(len(scan))
                cv2.imwrite(self.data_save_path + "/" + filename, image)
                writer.writerows([[stamp.to_sec(),
                                   filename,
                                   x,
                                   y,
                                   int(lbutton_down),
                                   key] +
                                  list(scan) + list(bump) + list(accel)])
                image_count += 1


if __name__ == '__main__':
    node = DataLogger()
    node.run()
