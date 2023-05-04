################################################################################
# Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# Leap Motion proprietary and confidential. Not for distribution.              #
# Use subject to the terms of the Leap Motion SDK Agreement available at       #
# https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# between Leap Motion and you, your company or other organization.             #
################################################################################

import Leap, sys, thread, time
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture
from leap_motion_msgs.msg import *
import rospy


class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']
    ros_pub = rospy.Publisher('/leap_motion', LeapData, queue_size=10)

    def on_init(self, controller):
        print("Initialized")

    def on_connect(self, controller):
        print("Connected")

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE)
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP)
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE)

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print("Disconnected")

    def on_exit(self, controller):
        print("Exited")

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        leap_msg = LeapData()
        leap_msg.header.stamp = rospy.Time.now()
        leap_msg.header.frame_id = "leap_motion"

        print("Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
              frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures())))

        # Get hands
        for hand in frame.hands:

            handType = "Left hand" if hand.is_left else "Right hand"

            hand_msg = Hand()
            hand_msg.id = hand.id
            hand_msg.hand_type = handType
            hand_msg.is_left = hand.is_left
            
            # Get the hand's normal vector and direction
            normal = hand.palm_normal
            direction = hand.direction

            # Get arm bone
            arm = hand.arm
            
            for i in range(3):
                hand_msg.hand_position[i] = hand.palm_position[i]
                hand_msg.arm_direction[i] = hand.arm.direction[i]
                hand_msg.wrist_position[i] = arm.wrist_position[i]
                hand_msg.elbow_position[i] = arm.elbow_position[i]

            hand_msg.hand_rotation[0] = direction.pitch * Leap.RAD_TO_DEG
            hand_msg.hand_rotation[1] = normal.roll * Leap.RAD_TO_DEG
            hand_msg.hand_rotation[2] = direction.yaw * Leap.RAD_TO_DEG

            finger_num = 0

            # Get fingers
            for finger in hand.fingers:
                finger_msg = Finger()

                finger_msg.id = finger.id
                finger_msg.finger_type = self.finger_names[finger.type]
                finger_msg.length = finger.length
                finger_msg.width = finger.width

                # Get bones
                for b in range(0, 4):
                    bone_msg = Bone()
                    bone = finger.bone(b)
                    bone_msg.bone_type = self.bone_names[bone.type]
                    for j in range(3):
                        bone_msg.start_position[j] = bone.prev_joint[j]
                        bone_msg.end_position[j] = bone.next_joint[j]
                        bone_msg.direction[j] = bone.direction[j]

                    finger_msg.bones[b] = bone_msg

                hand_msg.fingers[finger_num] = finger_msg
                finger_num += 1
            leap_msg.hands.append(hand_msg)

        # Get tools
        for tool in frame.tools:

            print("  Tool id: %d, position: %s, direction: %s" % (
                tool.id, tool.tip_position, tool.direction))

        # Get gestures
        for gesture in frame.gestures():
            if gesture.type == Leap.Gesture.TYPE_CIRCLE:
                circle = CircleGesture(gesture)

                # Determine clock direction using the angle between the pointable and the circle normal
                if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI/2:
                    clockwiseness = "clockwise"
                else:
                    clockwiseness = "counterclockwise"

                # Calculate the angle swept since the last frame
                swept_angle = 0
                if circle.state != Leap.Gesture.STATE_START:
                    previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
                    swept_angle =  (circle.progress - previous_update.progress) * 2 * Leap.PI

                print("  Circle id: %d, %s, progress: %f, radius: %f, angle: %f degrees, %s" % (
                        gesture.id, self.state_names[gesture.state],
                        circle.progress, circle.radius, swept_angle * Leap.RAD_TO_DEG, clockwiseness))

            if gesture.type == Leap.Gesture.TYPE_SWIPE:
                swipe = SwipeGesture(gesture)
                print("  Swipe id: %d, state: %s, position: %s, direction: %s, speed: %f" % (
                        gesture.id, self.state_names[gesture.state],
                        swipe.position, swipe.direction, swipe.speed))

            if gesture.type == Leap.Gesture.TYPE_KEY_TAP:
                keytap = KeyTapGesture(gesture)
                print("  Key Tap id: %d, %s, position: %s, direction: %s" % (
                        gesture.id, self.state_names[gesture.state],
                        keytap.position, keytap.direction ))

            if gesture.type == Leap.Gesture.TYPE_SCREEN_TAP:
                screentap = ScreenTapGesture(gesture)
                print("  Screen Tap id: %d, %s, position: %s, direction: %s" % (
                        gesture.id, self.state_names[gesture.state],
                        screentap.position, screentap.direction ))

        if not (frame.hands.is_empty and frame.gestures().is_empty):
            print("")
        
        self.ros_pub.publish(leap_msg)

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"

def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    rospy.init_node('leap_motion_publisher')

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print("Press Enter to quit...")
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()

