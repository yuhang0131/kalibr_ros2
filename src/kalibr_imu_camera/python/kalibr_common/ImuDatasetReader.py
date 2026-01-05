# ROS2 imports
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os
import sm
import numpy as np
import pylab as pl
import aslam_cv as acv

class BagImuDatasetReaderIterator(object):
    def __init__(self,dataset,indices=None):
        self.dataset = dataset
        if indices is None:
            self.indices = np.arange(dataset.numMessages())
        else:
            self.indices = indices
        self.iter = self.indices.__iter__()
    def __iter__(self):
        return self
    def next(self):
        # required for python 2.x compatibility
        idx = next(self.iter)
        return self.dataset.getMessage(idx)
    def __next__(self):
        idx = next(self.iter)
        return self.dataset.getMessage(idx)

class BagImuDatasetReader(object):
    def __init__(self, bagfile, imutopic, bag_from_to=None, perform_synchronization=False):
        self.bagfile = bagfile
        self.topic = imutopic
        self.perform_synchronization = perform_synchronization
        
        if imutopic is None:
            raise RuntimeError("Please pass in a topic name referring to the imu stream in the bag file\n{0}".format(bagfile))

        # ROS2: Open bag using rosbag2_py
        storage_options = StorageOptions(uri=bagfile, storage_id='sqlite3')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr')
        
        bag_reader = SequentialReader()
        bag_reader.open(storage_options, converter_options)
        
        # Read all messages and build index
        self.messages = []
        msg_type_str = None
        
        while bag_reader.has_next():
            (topic, data, timestamp) = bag_reader.read_next()
            if topic == imutopic:
                self.messages.append((topic, data, timestamp))
                if msg_type_str is None:
                    # Get message type from topic metadata
                    topic_types = bag_reader.get_all_topics_and_types()
                    for t in topic_types:
                        if t.name == imutopic:
                            msg_type_str = t.type
                            break
        
        if len(self.messages) == 0:
            raise RuntimeError("Could not find topic {0} in {1}.".format(imutopic, bagfile))
        
        if msg_type_str is None:
            raise RuntimeError("Could not determine message type for topic {0}".format(imutopic))
        
        self.msg_type = get_message(msg_type_str)
        self.indices = np.arange(len(self.messages))
        
        #sort the indices by header.stamp
        self.indices = self.sortByTime(self.indices)
        
        #go through the bag and remove the indices outside the timespan [bag_start_time, bag_end_time]
        if bag_from_to:
            self.indices = self.truncateIndicesFromTime(self.indices, bag_from_to)
            
    #sort the ros messages by the header time not message time
    def sortByTime(self, indices):
        self.timestamp_corrector = sm.DoubleTimestampCorrector()
        timestamps = list()
        
        for idx in indices:
            topic, data, bag_timestamp = self.messages[idx]
            msg = deserialize_message(data, self.msg_type)
            # ROS2: timestamp is in nanoseconds
            timestamp = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            timestamps.append(timestamp)
            
            if self.perform_synchronization:
                self.timestamp_corrector.correctTimestamp(
                    msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    bag_timestamp * 1e-9
                )
        
        sorted_tuples = sorted(zip(timestamps, indices))
        sorted_indices = [tuple_value[1] for tuple_value in sorted_tuples]
        return sorted_indices
    
    def truncateIndicesFromTime(self, indices, bag_from_to):
        #get the timestamps
        timestamps = list()
        
        for idx in indices:
            topic, data, bag_timestamp = self.messages[idx]
            msg = deserialize_message(data, self.msg_type)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            timestamps.append(timestamp)

        bagstart = min(timestamps)
        baglength = max(timestamps) - bagstart
        print("bagstart", bagstart)
        print("baglength", baglength)
        
        #some value checking
        if bag_from_to[0] >= bag_from_to[1]:
            raise RuntimeError("Bag start time must be bigger than end time.")
        if bag_from_to[0] < 0.0:
            sm.logWarn("Bag start time of {0} s is smaller 0".format(bag_from_to[0]))
        if bag_from_to[1] > baglength:
            sm.logWarn("Bag end time of {0} s is bigger than the total length of {1} s".format(bag_from_to[1], baglength))

        #find the valid timestamps
        valid_indices = []
        for idx_pos, idx in enumerate(indices):
            timestamp = timestamps[idx_pos]
            if timestamp >= (bagstart + bag_from_to[0]) and timestamp <= (bagstart + bag_from_to[1]):
                valid_indices.append(idx)
        
        sm.logWarn("BagImuDatasetReader: truncated {0} / {1} messages.".format(len(indices) - len(valid_indices), len(indices)))
        
        return valid_indices
    
    def __iter__(self):
        # Reset the bag reading
        return self.readDataset()
    
    def readDataset(self):
        return BagImuDatasetReaderIterator(self, self.indices)

    def readDatasetShuffle(self):
        indices = self.indices.copy()
        np.random.shuffle(indices)
        return BagImuDatasetReaderIterator(self, indices)

    def numMessages(self):
        return len(self.indices)
    
    def getMessage(self, idx):
        topic, data, bag_timestamp = self.messages[idx]
        msg = deserialize_message(data, self.msg_type)
        
        if self.perform_synchronization:
            timestamp = acv.Time(self.timestamp_corrector.getLocalTime(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))
        else:
            timestamp = acv.Time(msg.header.stamp.sec, msg.header.stamp.nanosec)
        
        omega = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        alpha = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        
        return (timestamp, omega, alpha)
