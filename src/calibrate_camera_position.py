#!/usr/bin/env python
import rospy
import tf
import numpy as np
from scipy.optimize import least_squares

def calc_residuals(estimates, tag_to_cam, origin_to_right, origin_to_left, right_to_tag, left_to_tag):
    """ Calculates the residual error between the positions estimated by the bundle adjustment
    and the measured positions
    """

    num_iterations = len(tag_to_cam)

    


    apriltag_pose = estimates[:7]


    right_x_error = estimates[7::14] - origin_to_right[:, 0]
    right_y_error = estimates[8::14] - origin_to_right[:, 1]
    right_z_error = estimates[9::14] - origin_to_right[:, 2]
    left_x_error = estimates[14::14] - origin_to_left[:, 0]
    left_y_error = estimates[15::14] - origin_to_left[:, 1]
    left_z_error = estimates[16::14] - origin_to_left[:, 2]


    expected_right_to_tag = [pred_right_x, pred_right_y, pred_right_z]


    residuals = []
    residuals.extend(right_x_error)
    residuals.extend(right_y_error)
    residuals.extend(right_z_error)
    residuals.extend(left_x_error)
    residuals.extend(left_y_error)
    residuals.extend(left_z_error)

    return residuals






class calibrate_camera(object):
    def __init__(self):
        rospy.init_node("calibrate_camera_position")
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        rate = rospy.Rate(1.0)
        tag_to_cam, origin_to_right, origin_to_left, right_to_tag, left_to_tag = self.gather_data()

        try:
            self.tf_listener.waitForTransform('base', 'tag_83_right', rospy.Time(0), rospy.Duration(3.0))
            apriltag_init = self.tf_listener.lookupTransform('base', 'tag_83_right', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logerr(ex)
            return

        initial_estimate = []
        initial_estimate.extend(apriltag_init[0])
        initial_estimate.extend(apriltag_init[1])
        print len(initial_estimate)
        for i in range(len(origin_to_right)):
            initial_estimate.extend(origin_to_right[i])
            initial_estimate.extend(origin_to_left[i])
            print len(initial_estimate)

        initial_estimate = np.array(initial_estimate)

        residuals = calc_residuals(initial_estimate, tag_to_cam, origin_to_right, origin_to_left, right_to_tag, left_to_tag)
        print residuals

        # exit()

        # max_trans = -1.0 * np.inf * np.ones(3)
        # min_trans = np.inf * np.ones(3)

        while not rospy.is_shutdown():
            # quat = tf.transformations.quaternion_from_euler(-3.14, -1.57, 0)
            # self.tf_broadcaster.sendTransform((0, 0, 0), quat, rospy.Time.now(), "false_gripper", "tag_0")


            try:
                (trans, rot) = self.tf_listener.lookupTransform('tag_83', 'camera_link', rospy.Time(0))
                # max_trans = np.maximum(trans, max_trans)
                # min_trans = np.minimum(trans, min_trans)
                # print max_trans - min_trans
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logerr(ex)
                continue

            


            self.tf_broadcaster.sendTransform(trans, rot, rospy.Time.now(), "camera_link", "tag_83_right")


            # try:
            #     (trans, rot) = self.tf_listener.lookupTransform('torso', 'false_camera', rospy.Time(0))
            #     print "Got transform:"
            #     print trans
            #     print rot
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            #     rospy.logerr(ex)
            #     continue

            rate.sleep()

    def gather_data(self, num_points=5):
        tag_to_cams = []
        origin_to_rights = []
        origin_to_lefts = []
        right_to_tags = []
        left_to_tags = []
        for i in range(num_points):
            raw_input("Press enter when you have moved the arms to point at the apriltags.  Do not move the apriltags.")


            # Get camera positions
            try:
                tag_to_cam = self.tf_listener.lookupTransform('tag_83', 'camera_link', rospy.Time(0))
                origin_to_right = self.tf_listener.lookupTransform('base', 'right_hand_camera', rospy.Time(0))
                origin_to_left = self.tf_listener.lookupTransform('base', 'left_hand_camera', rospy.Time(0))
                right_to_tag = self.tf_listener.lookupTransform('right_hand_camera', 'tag_83_right', rospy.Time(0))
                left_to_tag = self.tf_listener.lookupTransform('left_hand_camera', 'tag_83_left', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                rospy.logerr(ex)
                continue


            # Converts from a tuple containing a length 3 and a length 4 array to a single array of length 7
            tag_to_cam[0].extend(tag_to_cam[1])
            tag_to_cams.append(tag_to_cam[0])
            origin_to_right[0].extend(origin_to_right[1])
            origin_to_rights.append(origin_to_right[0])
            origin_to_left[0].extend(origin_to_left[1])
            origin_to_lefts.append(origin_to_left[0])
            right_to_tag[0].extend(origin_to_left[1])
            right_to_tags.append(right_to_tag[0])
            left_to_tag[0].extend(origin_to_left[1])
            left_to_tags.append(left_to_tag[0])

            # data.extend(tag_to_cam[0])
            # data.extend(tag_to_cam[1])
            # data.extend(origin_to_right[0])
            # data.extend(origin_to_right[1])
            # data.extend(origin_to_left[0])
            # data.extend(origin_to_left[1])
            # data.extend(right_to_tag[0])
            # data.extend(right_to_tag[1])
            # data.extend(left_to_tag[0])
            # data.extend(left_to_tag[1])

            # print data

        return (np.array(tag_to_cams),
            np.array(origin_to_rights),
            np.array(origin_to_lefts), 
            np.array(right_to_tags), 
            np.array(left_to_tags))
        # return np.array(data)




if __name__ == '__main__':
    calibrate_camera()
