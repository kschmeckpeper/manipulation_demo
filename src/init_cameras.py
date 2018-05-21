#!/usr/bin/env python

import os
import rospy
def main():
    # Close all cameras
    os.system('rosrun baxter_tools camera_control.py -c right_hand_camera')
    os.system('rosrun baxter_tools camera_control.py -c left_hand_camera')

    # We have to close the head camera last because we want it to stay closed
    os.system('rosrun baxter_tools camera_control.py -c head_camera')


    # Reopen Left and right cameras at the desired resolution
    os.system('rosrun baxter_tools camera_control.py -o right_hand_camera -r 1280x800')
    os.system('rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800')

if __name__ == '__main__':
    main()