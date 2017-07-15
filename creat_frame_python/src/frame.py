#!/usr/bin/env python
import roslib
roslib.load_manifest('creat_frame_python')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    #while not rospy.is_shutdown():
    br.sendTransform((0.0, 2.0, 0.0),(0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "object", "odom_combined")
    rate.sleep()
    print "done...1"


    # create object frame
#    def create_object_frame(self):
#        pose = geometry_msgs.msg.PoseStamped()

#        quat = np.squeeze(np.array(
#            tf.transformations.quaternion_from_euler(self.one_orientation_list[x], self.one_orientation_list[y],
                                                     self.one_orientation_list[z])))

#        broadcaster = static_transform.StaticTransformBroadcaster()
#        static_transformStamped = geometry_msgs.msg.TransformStamped()

#        static_transformStamped.header.stamp = rospy.Time.now()
#        static_transformStamped.header.frame_id = "world"
#      static_transformStamped.child_frame_id = "object"

#       static_transformStamped.transform.translation.x = float(self.one_pose_list[x]);
#       pose.pose.position.x = float(self.one_pose_list[x])
#       static_transformStamped.transform.translation.y = float(self.one_pose_list[y]);
#       pose.pose.position.y = float(self.one_pose_list[y])
#       static_transformStamped.transform.translation.z = float(self.one_pose_list[z]);
#       pose.pose.position.z = float(self.one_pose_list[z])

#        static_transformStamped.transform.rotation.x = quat[x];
#        pose.pose.orientation.x = quat[x]
#        static_transformStamped.transform.rotation.y = quat[y];
#        pose.pose.orientation.y = quat[y]
#        static_transformStamped.transform.rotation.z = quat[z];
#        pose.pose.orientation.z = quat[z]
#        static_transformStamped.transform.rotation.w = quat[w];
#        pose.pose.orientation.w = quat[w]

#        broadcaster.sendTransform(static_transformStamped)

#        pose.header.frame_id = 'world'
#        pose.header.stamp = rospy.Time(0)
#        return pose




