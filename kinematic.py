#!/usr/bin/python

import numpy as np
import PyKDL as kdl
import rospy
from sensor_msgs.msg import JointState
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import Robot
import tf.transformations

def create_kinematic_chain_from_robot_description(base_link="base_link", end_link="gripper", urdf_file=None, robot_description="/robot_description"):
    if urdf_file is None:
        robot = Robot.from_parameter_server(key=robot_description)
    else:
        fileObj = file(urdf_file, 'r')
        robot = Robot.from_xml_string(fileObj.read())
        fileObj.close()
    return KinematicChain(robot, base_link, end_link)


class KinematicChain(object):
    def __init__(self):
        rospy.loginfo("KinematicChain: create default constructor")

    def __init__(self, urdf, base_link, end_link):
        [tree_ok, robot_tree_] = treeFromUrdfModel(urdf)

        if not tree_ok:
            rospy.logerr("KinematicChain: URDF tree is not parsed properly")
            raise Exception("KinematicChain: URDF tree is not parsed properly")

        self.robot_tree_ = robot_tree_
        self.urdf_model_ = urdf
        self.kin_chain_ = robot_tree_.getChain(base_link, end_link)
        self.base_link_ = base_link
        self.end_link_ = end_link

        self.joints_name_ = self.get_joints_name()
        self.joints_type_ = self.get_joints_type()
        self.links_name_ = self.get_links_name()
        self.num_joints_ = len(self.joints_name_)
        self.forward_kin_ = kdl.ChainFkSolverPos_recursive(self.kin_chain_)

        self.joint_states = [0., 0., 0., 0., 0., 0.];
        rospy.Subscriber("/arm/joint_states", JointState, self.jointStateCallback)

        endeffector_frame = kdl.Frame()
        #print endeffector_frame


        #print "kin_tree_:",self.kin_chain

        #print "joints: ", self.joints_name_
        #print "links: ", self.links_name_
        #print "joint_type: ", self.joints_type_
        #print self.create_homo_matrix("arm_shoulder_pan_joint")
        #print self.create_homo_matrix("arm_shoulder_lift_joint")
        #print self.create_homo_matrix("arm_elbow_joint")
        #print self.create_homo_matrix("arm_wrist_1_joint")
        #print self.create_homo_matrix("arm_wrist_2_joint")
        #print self.create_homo_matrix("arm_wrist_3_joint")
        #print "compute_transformation_between_joint: ", self.compute_transformation_between_joint("arm_wrist_1_joint", "arm_wrist_3_joint")

        #self.forward_kinematics("arm_wrist_1_joint", "arm_wrist_3_joint")
        self.forward_kinematics()

    ##
    # @return List of joint names in the urdf_model (kinematic_chain)
    def get_joints_name(self, links=False, fixed=False):
        return self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=links, fixed=fixed)
    ##
    # @return List of joint type in the urdf_model (kinematic_chain)
    def get_joints_type(self, links=False, fixed=False):
        joints_type = []
        model_description = self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=links, fixed=fixed)
        for it in model_description:
            joint = self.urdf_model_.joint_map[it]
            joints_type.append(joint.type)

        return joints_type

    ##
    # @return List of links names in the urdf_model (kinematic_chain)
    def get_links_name(self, joints=False, fixed=True):
        links_name = []
        links_name = self.urdf_model_.get_chain(self.base_link_, self.end_link_, joints=joints, fixed=fixed)
        return links_name

    ##
    # @param Name of joint of which transformation needed
    # @return 4x4 homogeneous transformation
    def create_homo_matrix(self, joint_name):
        model_description = self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=False, fixed=False)
        for it in model_description:
            joint = self.urdf_model_.joint_map[it]
            if joint.name == joint_name:
                angle =  joint.origin.rpy
                pose = joint.origin.xyz
                continue
        homo_matrix = tf.transformations.compose_matrix(angles=angle, translate=pose)
        #print homo_matrix
        return homo_matrix

    # still error how to calculate diff of two homo matrix
    def compute_transformation_between_joint(self, joint_name1, joint_name2):
        #q1 = self.create_homo_matrix(joint_name1)
        #q2 = self.create_homo_matrix(joint_name2)
        #diff_mat = np.array(q1) - np.array(q2)
        #print diff_mat
        #diff_mod = np.mod(diff_mat, 2 * np.pi)
        #diff_alt = diff_mod - 2 * np.pi

        model_description = self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=False, fixed=False)
        angle = []
        pose = []
        for it in model_description:
            joint = self.urdf_model_.joint_map[it]
            if joint.name == joint_name1 or joint.name == joint_name2:
                angle.append(joint.origin.rpy)
                pose.append(joint.origin.xyz)
        diff_pose = np.array(pose[1]) - np.array(pose[0])
        diff_angle = np.array(angle[1]) - np.array(angle[0])
        homo_matrix = tf.transformations.compose_matrix(angles=diff_angle, translate=diff_pose)
        print homo_matrix

    ##
    # @return get joint angles
    def get_joint_angle(self):
        return self.joint_states

    # receives the joint states
    def jointStateCallback(self, msg):
        for i in range(0, len(msg.position)):
            self.joint_states[i] = msg.position[i]

    def forward_kinematics(self, frame_name1=None, frame_name2=None):
        q = self.get_joint_angle()
        base_no = 3
        end_no = 5
        if frame_name1 is None:
            base_no = 0
        if frame_name2 is None:
            end_no = 5

        base_trans = self._do_kdl_fk(q,base_no)
        end_trans = self._do_kdl_fk(q,end_no)

        print base_trans** -1 * end_trans


    def _do_kdl_fk(self, q, joint_number):
        endeffec_frame = kdl.Frame()
        kinematics_status = self.forward_kin_.JntToCart(joint_list_to_kdl(q),endeffec_frame, joint_number)
        if kinematics_status >= 0:
            p = endeffec_frame.p
            M = endeffec_frame.M
            return np.mat([[M[0,0], M[0,1], M[0,2], p.x()],
                           [M[1,0], M[1,1], M[1,2], p.y()],
                           [M[2,0], M[2,1], M[2,2], p.z()],
                           [     0,      0,      0,     1]])
        else:
            return None

def joint_list_to_kdl(q):
    if q is None:
        return None
    if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]
    q_kdl = kdl.JntArray(len(q))
    for i, q_i in enumerate(q):
        q_kdl[i] = q_i
    return q_kdl

if __name__ == '__main__':
    create_kinematic_chain_from_robot_description()




























#!/usr/bin/python

import numpy as np
import PyKDL as kdl
import rospy
from sensor_msgs.msg import JointState
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import Robot
import tf.transformations

def create_kinematic_chain_from_robot_description(base_link="base_link", end_link="gripper", urdf_file=None, robot_description="/robot_description"):
    if urdf_file is None:
        robot = Robot.from_parameter_server(key=robot_description)
    else:
        fileObj = file(urdf_file, 'r')
        robot = Robot.from_xml_string(fileObj.read())
        fileObj.close()
    return KinematicChain(robot, base_link, end_link)


class KinematicChain(object):

    def __init__(self):
        rospy.loginfo("KinematicChain: create default constructor")

    def __init__(self, urdf, base_link="base_link", end_link="gripper"):
        [tree_ok, robot_tree_] = treeFromUrdfModel(urdf)

        if not tree_ok:
            rospy.logerr("KinematicChain: URDF tree is not parsed properly")
            raise Exception("KinematicChain: URDF tree is not parsed properly")

        self.robot_tree_ = robot_tree_
        self.urdf_model_ = urdf
        self.kin_chain_ = robot_tree_.getChain(base_link, end_link)
        self.base_link_ = base_link
        self.end_link_ = end_link

        self.joints_name_ = self.get_joints_name()
        self.joints_type_ = self.get_joints_type()
        self.links_name_ = self.get_links_name()
        self.num_joints_ = len(self.joints_name_)
        self.forward_kin_ = kdl.ChainFkSolverPos_recursive(self.kin_chain_)

        self.joint_states = [0., 0., 0., 0., 0., 0.];
        rospy.Subscriber("/arm/joint_states", JointState, self.jointStateCallback)

        #print endeffector_frame


        #print "kin_tree_:",self.kin_chain

        #print "joints: ", self.joints_name_
        #print "links: ", self.links_name_
        #print "joint_type: ", self.joints_type_
        #print "compute_transformation_between_joint: ", self.compute_transformation_between_joint("arm_wrist_1_joint", "arm_wrist_3_joint")

        print self.forward_kinematics("arm_wrist_3_joint", "arm_wrist_2_joint")
        #self.forward_kinematics()

    ##
    # @return List of joint names in the urdf_model (kinematic_chain)
    def get_joints_name(self, links=False, fixed=False):
        return self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=links, fixed=fixed)
    ##
    # @return List of joint type in the urdf_model (kinematic_chain)
    def get_joints_type(self, links=False, fixed=False):
        joints_type = []
        model_description = self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=links, fixed=fixed)
        for it in model_description:
            joint = self.urdf_model_.joint_map[it]
            joints_type.append(joint.type)

        return joints_type

    ##
    # @return List of links names in the urdf_model (kinematic_chain)
    def get_links_name(self, joints=False, fixed=True):
        links_name = []
        links_name = self.urdf_model_.get_chain(self.base_link_, self.end_link_, joints=joints, fixed=fixed)
        return links_name

    ##
    # @param Name of joint of which transformation needed
    # @return 4x4 homogeneous transformation
    def create_homo_matrix(self, joint_name):
        model_description = self.urdf_model_.get_chain(self.base_link_, self.end_link_, links=False, fixed=False)
        for it in model_description:
            joint = self.urdf_model_.joint_map[it]
            if joint.name == joint_name:
                angle =  joint.origin.rpy
                pose = joint.origin.xyz
                continue
        homo_matrix = tf.transformations.compose_matrix(angles=angle, translate=pose)
        return homo_matrix

    ##
    # @return get joint angles
    def get_joint_angle(self):
        return self.joint_states

    # receives the joint states
    def jointStateCallback(self, msg):
        for i in range(0, len(msg.position)):
            self.joint_states[i] = msg.position[i]


    ##
    # Forward kinematics on the given joint angles, returning the location of the
    # end_link in the base_link's frame.
    # @param end_link Name of the link the pose should be obtained for.
    # @param base_link Name of the root link frame the end_link should be found in.
    # @return 4x4 numpy.mat homogeneous transformation
    def forward_kinematics(self, end_joint_name=None, base_joint_name=None):
        q = self.get_joint_angle()

        joints_name = self.joints_name_
        if end_joint_name is None:
            end_joint = self.chain.getNrOfSegments()

        else:
            if end_joint_name in joints_name:
                end_joint = joints_name.index(end_joint_name)
                #print end_joint
            else:
                print "Target segment %s not in KDL chain" % end_joint_name
                return None

        if base_joint_name is None:
            base_joint = 0
        else:
            if base_joint_name in joints_name:
                base_joint = joints_name.index(base_joint_name)
                #print base_joint
            else:
                print "Base segment %s not in KDL chain" % base_joint_name
                return None

        base_trans = self._do_kdl_fk(q, base_joint)
        if base_trans is None:
            print "FK KDL failure on base transformation."

        end_trans = self._do_kdl_fk(q, end_joint)
        if end_trans is None:
            print "FK KDL failure on end transformation."
        return base_trans ** -1 * end_trans


    ##
    # @param Name of joint of which transformation needed
    # @return 4x4 homogeneous transformation
    def _do_kdl_fk(self, q, joint_number):
        endeffec_frame = kdl.Frame()
        kinematics_status = self.forward_kin_.JntToCart(self.create_joint_list(q),endeffec_frame, joint_number)
        if kinematics_status >= 0:
            p = endeffec_frame.p
            M = endeffec_frame.M
            return np.mat([[M[0,0], M[0,1], M[0,2], p.x()],
                           [M[1,0], M[1,1], M[1,2], p.y()],
                           [M[2,0], M[2,1], M[2,2], p.z()],
                           [     0,      0,      0,     1]])
        else:
            return None

    ##
    # @param q Name of joint of which transformation needed
    def create_joint_list(self,q):
        if q is None:
            return None
        if type(q) == np.matrix and q.shape[1] == 0:
            q = q.T.tolist()[0]
        q_kdl = kdl.JntArray(len(q))
        for i, q_i in enumerate(q):
            q_kdl[i] = q_i
        return q_kdl


    def write_list_form(self):
        list_joint = [[]]
        for it_joint in range(0, self.num_joints_):
            joint_name = self.joints_name_[it_joint]
            if it_joint is not 5:
                trans = self.forward_kinematics(self.joints_name_[it_joint+1], joint_name)
            else:
                trans = None
            trans_wrt_origin = self.forward_kinematics(end_joint_name=joint_name)
            list_joint.append((it_joint, joint_name, trans, trans_wrt_origin))
        return list_joint

    def print_list(self):
        for it in self.write_list_form():
            print it
            print "/n"

if __name__ == '__main__':
    create_kinematic_chain_from_robot_description()
    robot = Robot.from_parameter_server()
    kdl_chanin = KinematicChain(robot)
    #kdl_chanin.write_list_form()
    pose = kdl_chanin.forward_kinematics("arm_wrist_3_joint","arm_wrist_2_joint")
   # print "pose: ",pose

