import numpy as np
import time
import rospy
import argparse
import csv
from leap_motion_msgs.msg import *

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup
    
class LeapMotionTeleop:
    def __init__(self, reset_joints, record_data, data_save_dir):
        self.teleop = False
        self.leap_sub = rospy.Subscriber("/leap_motion", LeapData, self.callback)
        print("leap motion sensor subscribed")
        self.data_array = np.zeros((21))
        self.data_record = record_data
        self.data_save_dir = data_save_dir
        self.fa = FrankaArm()
        self.pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
        self.pub_id = 0
        self.last_delta_position = None

        if reset_joints:
            self.fa.reset_joints()

        self.robot_start_pose = self.fa.get_pose()
        self.hand_start_pose = None
        self.init_time = rospy.Time.now().to_time()

        if record_data:
            self.csv_file = open(self.data_save_dir, 'w')
            self.csv_writer = csv.writer(self.csv_file)
            # h1 = left hand, h2 = right hand, f1 = thumb, f2 = index finger
            header = ['timestamp','h1x', 'h1y', 'h1z','h1rx', 'h1ry', 'h1rz', 'h1f1x', 'h1f1y', 'h1f1z', 'h1f2x', 'h1f2y', 'h1f2z','h2x', 'h2y', 'h2z', 'h2f1x', 'h2f1y', 'h2f1z', 'h2f2x', 'h2f2y', 'h2f2z']
            self.csv_writer.writerow(header)

    def callback(self, data):
        self.data_array = np.zeros((21))

        for hand in data.hands:
            if hand.is_left:
                for i in range(3):
                    self.data_array[i] = hand.hand_position[i]
                    self.data_array[i+3] = hand.hand_rotation[i]
                
                for finger in hand.fingers:
                    if finger.finger_type == "Thumb":
                        for i in range(3):
                            self.data_array[i+6] = finger.bones[3].end_position[i]
                    elif finger.finger_type == "Index":
                        for i in range(3): 
                            self.data_array[i+9] = finger.bones[3].end_position[i]
            else:
                for i in range(3):
                    self.data_array[i+12] = hand.hand_position[i]
                
                for finger in hand.fingers:
                    if finger.finger_type == "Thumb":
                        for i in range(3):
                            self.data_array[i+15] = finger.bones[3].end_position[i]
                    elif finger.finger_type == "Index":
                        for i in range(3): 
                            self.data_array[i+18] = finger.bones[3].end_position[i]

        if self.data_record:
            timestamp = time.time()
            data = [timestamp] + self.data_array.tolist()
            self.csv_writer.writerow(data)

        if self.teleop:

            if self.data_array[0] == 0 and self.data_array[1] == 0 and self.data_array[2] == 0:
                print('Left hand is currently not visible. Stopping teleop.')
                self.stopTeleop()
            else:
                desired_delta_pose = (self.data_array[0:6] - self.hand_start_pose) * 0.001
                desired_delta_pose = [desired_delta_pose[2], desired_delta_pose[0], desired_delta_pose[1], desired_delta_pose[5], desired_delta_pose[3], desired_delta_pose[4]]
                delta_difference = np.clip(desired_delta_pose - self.last_delta_position, -0.02, 0.02)
                print(desired_delta_pose)
                timestamp = rospy.Time.now().to_time() - self.init_time
                traj_gen_proto_msg = PosePositionSensorMessage(
                    id=self.pub_id, timestamp=timestamp, 
                    position=self.robot_start_pose.translation + self.last_delta_position[0:3] + delta_difference[0:3], 
                    quaternion=self.robot_start_pose.quaternion
                )
                ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
                )

                self.last_delta_position += delta_difference
                self.pub_id += 1

                rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
                self.pub.publish(ros_msg)

    def startRecord(self):
        self.data_record = True

    def stopRecord(self):
        self.data_record = False

    def startTeleop(self):
        self.teleop = True
        self.init_time = rospy.Time.now().to_time()
        self.robot_start_pose = self.fa.get_pose()
        self.hand_start_pose = self.data_array[0:6]
        self.last_delta_position = np.zeros((6))
        if self.hand_start_pose[0] == 0 and self.hand_start_pose[1] == 0 and self.hand_start_pose[2] == 0:
            self.teleop = False
            print('Left hand is currently not visible. Not starting teleop yet.')
        else:
            self.fa.goto_pose(self.robot_start_pose, duration=1000, dynamic=True, buffer_time=10)
            self.pub_id = 0
        

    def stopTeleop(self): 

        self.teleop = False

        term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - self.init_time, should_terminate=True)
        ros_msg = make_sensor_group_msg(
                    termination_handler_sensor_msg=sensor_proto2ros_msg(
                        term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
                    )
        self.pub.publish(ros_msg)

    def terminateRecord(self):
        self.csv_file.close()

    def get_curr_reading(self):
        return self.data_array


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--reset_joints', '-j', action='store_true')
    parser.add_argument('--record', '-r', action='store_true')
    parser.add_argument('--save_dir', '-s', default='')
    args = parser.parse_args()

    leap_motion_teleop = LeapMotionTeleop(args.reset_joints, args.record, args.save_dir)
    while(True):
        current_input = input()
        if current_input == 't':
            leap_motion_teleop.startTeleop()
        elif current_input == 's':
            leap_motion_teleop.stopTeleop()