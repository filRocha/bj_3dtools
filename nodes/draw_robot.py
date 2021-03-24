#!/usr/bin/env python
import rospy
import open3d as o3d
import numpy as np
import copy as cp

from bj_libraries.msg import DualQuaternionStamped

class Open3dTest():

    # constructor
    def __init__(self):

        # sends a message to the user
        rospy.loginfo("draw_robot node started.")

        # parameters
        p_loop_rate = 10
        p_frame_size = 0.4

        # initializing current pose attributes
        self.robot_pose = []
        self.mani_base_pose = []
        self.mani_tcp_pose = []
        self.setpoint_pose = []

        # initializing past pose attributes
        robot_pose_before = []
        mani_base_pose_before = []
        mani_tcp_pose_before = []
        setpoint_pose_before =[]
        poses_before = [robot_pose_before, mani_base_pose_before, mani_tcp_pose_before, setpoint_pose_before]

        # Open3d parameters
        self.vis = []
        f_robo = o3d.geometry.TriangleMesh.create_coordinate_frame(size=p_frame_size)
        f_mani_base = o3d.geometry.TriangleMesh.create_coordinate_frame(size=p_frame_size)
        f_mani_tcp = o3d.geometry.TriangleMesh.create_coordinate_frame(size=p_frame_size)
        f_setpoint = o3d.geometry.TriangleMesh.create_coordinate_frame(size=p_frame_size)
        frames = [f_robo, f_mani_base, f_mani_tcp, f_setpoint]

        # registering to subscribers
        sub_robot_pose = rospy.Subscriber('/rosi/cheat/rosi_pose', DualQuaternionStamped, self.cllbck_robot)
        sub_mani_base_pose = rospy.Subscriber('/rosi/cheat/mani_base_pose', DualQuaternionStamped, self.cllbck_mani_base)
        sub_mani_tcp_pose = rospy.Subscriber('/rosi/cheat/tcp_pose', DualQuaternionStamped, self.cllbck_tcp)
        sub_setpoint_pose = rospy.Subscriber('/sim/pose/dummy_1', DualQuaternionStamped, self.cllbck_setpoint)

        # creating the visualization
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()

        # defining loop frequency
        node_sleep_rate = rospy.Rate(p_loop_rate)

        # auxiliary flags
        flg_first_transform = True
        flg_first_run = True

        # referencing arrays arrays



        # puts node to spin in ROS framework
        while not rospy.is_shutdown():

            # condition for the first run when all data has been received
            if flg_first_run and self.check_income_data():

                # uncheck first run flag
                flg_first_run = False

                # add geometries to the visualization
                #self.vis.add_geometry(self.f_robo)
                self.vis_addframe(frames)

            # now run if there is incoming data
            elif self.check_income_data():

                rospy.loginfo("CP1")

                # updates poses reference list
                poses_now = [self.robot_pose, self.mani_base_pose, self.mani_tcp_pose, self.setpoint_pose]

                # updates open3d geometries
                frames, poses_before = self.frame_pose_update(frames, poses_now, poses_before, flg_first_transform)

                # conforming first transform flag
                if flg_first_transform:
                    flg_first_transform = False

                # updates the geometry
                self.vis_update(frames)

            # sleeps the node for a given rate
            node_sleep_rate.sleep()

    # topic callback function - robot pose
    def cllbck_robot(self, msg):
        self.robot_pose = msg

    # topic callback function - robot pose
    def cllbck_mani_base(self, msg):
        self.mani_base_pose = msg

    # topic callback function - robot pose
    def cllbck_tcp(self, msg):
        self.mani_tcp_pose = msg

    # topic callback function - robot pose
    def cllbck_setpoint(self, msg):
        self.setpoint_pose = msg

    # add frames to the visualizer
    def vis_addframe(self, frames):

        # insert frames in the visualizer
        for frame_i in frames:
            self.vis.add_geometry(frame_i)

    # update visualization
    def vis_update(self, frames):

        # update frames geometry
        for frame_i in frames:
            self.vis.update_geometry(frame_i)

        # update the rest
        self.vis.poll_events()
        self.vis.update_renderer()

    # discovers if all topics have received data
    # returns False if not and True if they all have received
    def check_income_data(self):
        return not((not self.robot_pose) or (not self.mani_base_pose) or (not self.mani_tcp_pose) or (not self.setpoint_pose))

    # Updates a frame pose
    # pose_n: Pose now (current pose)
    # pose_b: Pose before
    @staticmethod
    def frame_pose_update(frames, poses_now, poses_before, flg_prior_transform):

        if not flg_prior_transform: # in case that a prior transform has already occurred

            # obtaining before poses translations and rotations
            transl_before, rotm_before = Open3dTest.translrotm_from_msg(poses_before)

            # performing transform
            i = 0
            for transl, rotm in zip(transl_before, rotm_before):
                frames[i].translate(-transl)
                frames[i].rotate(rotm.transpose())
                i += 1

        # obtaining current poses translations and rotations
        transl_now, rotm_now = Open3dTest.translrotm_from_msg(poses_now)

        # performs current transform
        i = 0
        for transl, rotm in zip(transl_now, rotm_now):
            frames[i].translate(transl)
            frames[i].rotate(rotm)
            i += 1

        # updates the last time-step poses
        poses_before = cp.deepcopy(poses_now)

        return frames, poses_before

    # computes a transforms matrix given the received message
    @staticmethod
    def transform_from_msg(msg):

        # computing the rotation matrix
        rotm = o3d.geometry.get_rotation_matrix_from_quaternion([msg.wp, msg.xp, msg.yp, msg.zp])

        # returning the mounted homogeneous transform matrix
        return np.hstack((np.vstack((rotm, [0, 0, 0])), [[msg.xd], [msg.yd], [msg.zd], [1]]))

    # computes a translation vector and a rotation matrix given received message
    @staticmethod
    def translrotm_from_msg(poses):

        if isinstance(poses, list):  # in case of receiving message is a list

            # extracts the translation
            transl = []
            rotm = []
            for pose in poses:

                # computes translation
                transl.append(np.array([pose.xd, pose.yd, pose.zd]))

                # computes rotation matrix
                rotm.append(o3d.geometry.get_rotation_matrix_from_quaternion(
                    [pose.wp, pose.xp, pose.yp, pose.zp]))

        else:   # in case receiving message is an element

            # assigns to a local variable reference
            pose = poses

            # computes translation
            transl = np.array([pose.xd, pose.yd, pose.zd])

            # computes translation matrix
            rotm = o3d.geometry.get_rotation_matrix_from_quaternion(
                [pose.wp, pose.xp, pose.yp, pose.zp])

        return transl, rotm


# instantiate the node
if __name__ == '__main__':

    # initialize the node
    rospy.init_node('draw_robot',anonymous=True)

    # instantiate the node class
    try:
        node_obj = Open3dTest()
    except rospy.ROSInternalException: pass