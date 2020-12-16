#!/usr/bin/env python
import rospy
import random
import geometry_msgs.msg 
import tf2_msgs.msg
import os
import tf.transformations

# the noise parameter
odom_t_noise = 0.1 # translation
odom_r_noise = 0.05 # rotation
tags_t_noise = 0.1
tags_r_noise = 0.05

goal_pub = rospy.Publisher('goal', tf2_msgs.msg.TFMessage, queue_size = 10)
odom_pub = rospy.Publisher('odom', tf2_msgs.msg.TFMessage, queue_size = 10)
tags_pub = rospy.Publisher('tags', tf2_msgs.msg.TFMessage, queue_size = 10)

def goal(data):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "robot"
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = "goal"
    t.transform.translation.x = -2.0
    t.transform.translation.y = -2.0
    t.transform.translation.z = 0.0

    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    temp_data = tf2_msgs.msg.TFMessage([t])

    # pub = rospy.Publisher('goal', tf2_msgs.msg.TFMessage, queue_size = 10)
    goal_pub.publish(temp_data)

def callback(data):
    ### TODO: data.transforms[0].header
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    if data.transforms[0].child_frame_id == "base_link":
        temp_data = data
	
        temp_data.transforms[0].transform.translation.x = data.transforms[0].transform.translation.x + random.random() * odom_t_noise - odom_t_noise / 2
        temp_data.transforms[0].transform.translation.y = data.transforms[0].transform.translation.y + random.random() * odom_t_noise - odom_t_noise / 2
        temp_data.transforms[0].transform.translation.z = random.random() * 0.01 - 0.005 # z-axis

        x = temp_data.transforms[0].transform.rotation.x
        y = temp_data.transforms[0].transform.rotation.y
        z = temp_data.transforms[0].transform.rotation.z
        w = temp_data.transforms[0].transform.rotation.w

        q_origin = (x, y, z, w)
        q_rotate = tf.transformations.quaternion_from_euler(0, 0, random.random() * odom_r_noise - odom_r_noise / 2)
        q_new = tf.transformations.quaternion_multiply(q_rotate, q_origin)

        temp_data.transforms[0].transform.rotation.x = q_new[0]
        temp_data.transforms[0].transform.rotation.y = q_new[1]
        temp_data.transforms[0].transform.rotation.z = q_new[2]
        temp_data.transforms[0].transform.rotation.w = q_new[3]

        odom_pub.publish(temp_data)

    elif data.transforms[0].child_frame_id[:3] == "tag":
        x = data.transforms[0].transform.rotation.x
        y = data.transforms[0].transform.rotation.y
        z = data.transforms[0].transform.rotation.z
        w = data.transforms[0].transform.rotation.w
        
        # rotation about x_axis to make tag pose z-up=true
        q_matrix = tf.transformations.quaternion_matrix((x, y, z, w))
        q_matrix[1,:] = - q_matrix[1,:]
        q_matrix[2,:] = - q_matrix[2,:]
        q_matrix[0,3] = data.transforms[0].transform.translation.x
        q_matrix[1,3] = -data.transforms[0].transform.translation.y
        q_matrix[2,3] = -data.transforms[0].transform.translation.z
        q_matrix[3,3] = 1.0

        q_origin = tf.transformations.quaternion_from_matrix(q_matrix)
        t = tf.transformations.translation_from_matrix(q_matrix)

        temp_data = data
        temp_data.transforms[0].header.frame_id = "robot"
        temp_data.transforms[0].child_frame_id = data.transforms[0].child_frame_id[4:]

        temp_data.transforms[0].transform.translation.x = t[0] + random.random() * tags_t_noise - tags_t_noise / 2
        temp_data.transforms[0].transform.translation.y = t[1] + random.random() * tags_t_noise - tags_t_noise / 2
        temp_data.transforms[0].transform.translation.z = t[2] + random.random() * 0.01 - 0.005



        q_rotate = tf.transformations.quaternion_from_euler(0, 0, random.random() * tags_r_noise - tags_r_noise / 2)
        q_new = tf.transformations.quaternion_multiply(q_rotate, q_origin)



        temp_data.transforms[0].transform.rotation.x = q_new[0]
        temp_data.transforms[0].transform.rotation.y = q_new[1]
        temp_data.transforms[0].transform.rotation.z = q_new[2]
        temp_data.transforms[0].transform.rotation.w = q_new[3]

        # e1 = tf.transformations.euler_from_quaternion(q_new)
        # print "e1 ", e1
        

        tags_pub.publish(temp_data)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("tf", tf2_msgs.msg.TFMessage, goal)
    rospy.Subscriber("tf", tf2_msgs.msg.TFMessage, callback)
    rospy.spin()

if __name__ == '__main__':
    x = str(random.random() * 18 - 9.0)
    y = str(random.random() * 18 - 9.0)
    # random position about initial pose
    cmd = "rosrun gazebo_ros spawn_model -urdf -param /robot_description -model example  -x " + x + " -y " + y + " -z 0.0"
    returned_value = os.system(cmd)
    listener()

