import rospy
import numpy as np
import datetime
import time 
import csv
import signal
from leap_motion_msgs.msg import *
import argparse 

class GracefulExiter():

    def __init__(self):
        self.state = False
        signal.signal(signal.SIGINT, self.change_state)

    def change_state(self, signum, frame):
        print("exit flag set to True (repeat to exit now)")
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        self.state = True

    def exit(self):
        return self.state

class LeapMotionLogger:
    def __init__(self, record_data, data_save_dir):
        self.leap_sub = rospy.Subscriber("/leap_motion", LeapData, self.callback)
        print("leap motion sensor subscribed")
        self.data_array = np.zeros((18))
        self.data_record = record_data
        self.data_save_dir = data_save_dir

        if record_data:
            self.csv_file = open(self.data_save_dir, 'w')
            self.csv_writer = csv.writer(self.csv_file)
            # h1 = left hand, h2 = right hand, f1 = thumb, f2 = index finger
            header = ['timestamp','h1x', 'h1y', 'h1z', 'h1f1x', 'h1f1y', 'h1f1z', 'h1f2x', 'h1f2y', 'h1f2z','h2x', 'h2y', 'h2z', 'h2f1x', 'h2f1y', 'h2f1z', 'h2f2x', 'h2f2y', 'h2f2z']
            self.csv_writer.writerow(header)

    def callback(self, data):
        self.data_array = np.zeros((18))

        for hand in data.hands:
            if hand.is_left:
                for i in range(3):
                    self.data_array[i] = hand.hand_position[i]
                
                for finger in hand.fingers:
                    if finger.finger_type == "Thumb":
                        for i in range(3):
                            self.data_array[i+3] = finger.bones[3].end_position[i]
                    elif finger.finger_type == "Index":
                        for i in range(3): 
                            self.data_array[i+6] = finger.bones[3].end_position[i]
            else:
                for i in range(3):
                    self.data_array[i+9] = hand.hand_position[i]
                
                for finger in hand.fingers:
                    if finger.finger_type == "Thumb":
                        for i in range(3):
                            self.data_array[i+12] = finger.bones[3].end_position[i]
                    elif finger.finger_type == "Index":
                        for i in range(3): 
                            self.data_array[i+15] = finger.bones[3].end_position[i]

        if self.data_record:
            timestamp = time.time()
            data = [timestamp] + self.data_array.tolist()
            self.csv_writer.writerow(data)

    def startRecord(self):
        self.data_record = True

    def stopRecord(self):
        self.data_record = False

    def terminateRecord(self):
        self.csv_file.close()

    def get_curr_reading(self):
        return self.data_array


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--record', '-r', action='store_true')
    parser.add_argument('--save_dir', '-s', default='')
    args = parser.parse_args()

    rospy.init_node("leap_motion_logger")
    leap_motion_logger = LeapMotionLogger(args.record, args.save_dir)

    flag = GracefulExiter()

    while(True):
        print(leap_motion_logger.get_curr_reading())
        time.sleep(0.033)

        if flag.exit():
            leap_motion_logger.terminateRecord()
            break