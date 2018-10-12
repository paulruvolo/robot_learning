#!/usr/bin/env python

import rospy
import rospkg
import csv
import cv2
import numpy as np
import cPickle as pickle


class LearnBump(object):
    def __init__(self):
        self.target_size = (224, 224)
        r = rospkg.RosPack()
        self.data_dir = rospy.get_param('~data_dir', 'bump_final')
        self.data_load_path = r.get_path('data_processing_utilities') + \
            '/data/' + self.data_dir
        self.load_metadata()
        self.load_images()

    def load_metadata(self):
        metadata_file = self.data_load_path + '/metadata.csv'
        with open(metadata_file) as f:
            r = csv.DictReader(f)
            data_raw = [row for row in r]

            self.image_files = []
            self.data = []
            for row in data_raw:
                keys_no_filename = set(row.keys()) - set(['image_file_name'])
                new_value = {key: float(row[key]) for key in keys_no_filename}
                self.data.append(new_value)
                self.image_files.append(row['image_file_name'])

    def load_images(self):
        self.images = [cv2.resize(cv2.imread(self.data_load_path + '/' + f),
                                  self.target_size)
                       for f in self.image_files]

    def create_datasets(self):
        n_train = len(self.images) // 2
        np.save(self.data_load_path +
                "/images_train_" +
                str(self.target_size[0]) +
                "_" +
                str(self.target_size[1]) +
                ".npy",
                np.asarray(self.images[:n_train]))
        np.save(self.data_load_path +
                "/images_test_" +
                str(self.target_size[0]) +
                "_" +
                str(self.target_size[1]) +
                ".npy",
                np.asarray(self.images[n_train:]))

        # save the training and test labels for subsequent learning
        with open(self.data_load_path + "/labels_train.pkl", 'wt') as f:
            pickle.dump(self.data[:n_train], f)

        with open(self.data_load_path + "/labels_test.pkl", 'wt') as f:
            pickle.dump(self.data[n_train:], f)


if __name__ == '__main__':
    node = LearnBump()
    node.create_datasets()
