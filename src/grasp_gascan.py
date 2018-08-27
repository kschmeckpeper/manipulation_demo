import sys
import rospy
import moveit_commander
import geometry_msgs.msg

import tf2_ros
import tf2_geometry_msgs

import baxter_interface
import tf

from std_srvs.srv import *

class GraspGascan(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('grasp_gascan', anonymous=True)

        self.home_pose = [0.5, 0.4, 0.3]

        self.gripper = baxter_interface.Gripper('left')
        self.gripper.command_position(100, block=True)

        robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("left_arm")
        self.group.set_goal_joint_tolerance(0.05)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # self.grasp(None)
        self.subscriber = rospy.Subscriber("/smoothed_object_pose_array", geometry_msgs.msg.PoseArray, self.grasp)

        s = rospy.Service('grasp_gascan', Empty, self.trigger_grasp)
        rospy.spin()

    def trigger_grasp(self, req):
        self.subscriber = rospy.Subscriber("/smoothed_object_pose_array", geometry_msgs.msg.PoseArray, self.grasp)
        return EmptyResponse()

    def grasp(self, msg):
        camera_frame_pose = geometry_msgs.msg.PoseStamped()
        camera_frame_pose.header = msg.header
        camera_frame_pose.pose = msg.poses[0]
        print camera_frame_pose

        transform = self.tf_buffer.lookup_transform('base',
                                       'head_camera_fixed',
                                       msg.header.stamp,
                                       rospy.Duration(1.0))

        pose_transformed = tf2_geometry_msgs.do_transform_pose(camera_frame_pose, transform)
        print pose_transformed

        roll, pitch, yaw = tf.transformations.euler_from_quaternion([0, 0, 1, 0])
        roll += 0
        pitch += -1.57
        yaw += 1.57
        print roll, pitch, yaw
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        print quat

        # pose = [0.5, 0.2, 0.2]
        pose = [pose_transformed.pose.position.x, pose_transformed.pose.position.y, 0.4]
        if not self._move_arm_to_pose(pose, quat):
            rospy.logwarn("Planning failed. Exiting")
            return

        pose[2] -= 0.2
        self._move_arm_to_pose(pose, quat)

        self.gripper.command_position(0, block=True)

        pose[2] += 0.2
        self._move_arm_to_pose(pose, quat)

        rospy.sleep(2)
        pose[2] -= 0.2
        self._move_arm_to_pose(pose, quat)
        self.gripper.command_position(100, block=True)

        pose[2] += 0.2
        self._move_arm_to_pose(pose, quat)
        self._move_arm_to_pose(self.home_pose, quat)
        print "Finished"
        self.subscriber.unregister()

    def _move_arm_to_pose(self, pose, quat):
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = quat[0]
        pose_target.orientation.x = quat[1]
        pose_target.orientation.y = quat[2] # 1.0
        pose_target.orientation.z = quat[3]
        pose_target.position.x = pose[0]
        pose_target.position.y = pose[1]
        pose_target.position.z = pose[2]
        self.group.set_pose_target(pose_target)

        plan = self.group.plan()

        if len(plan.joint_trajectory.points) == 0:
            return False

        self.group.go(wait=True)
        execute = self.group.execute(plan)
        self.group.clear_pose_targets()
        return True

if __name__ == '__main__':
    GraspGascan()