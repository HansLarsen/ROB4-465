#! /usr/bin/env python
import rospy 
import rospkg 
from gazebo_msgs.srv import SetModelConfiguration


if __name__ == '__main__':

    rospy.init_node('robot_pos_fix')
    rospy.sleep(5)

    rospy.wait_for_service('/gazebo/set_model_configuration')
    try:
        service = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        req = SetModelConfiguration()
        req.model_name = 'j2n6s300'
        req.urdf_param_name = ''
        req.joint_names = ["j2n6s300_joint_1","j2n6s300_joint_2","j2n6s300_joint_3","j2n6s300_joint_4","j2n6s300_joint_5","j2n6s300_joint_6"]
        req.joint_positions = [-2.3960643184528507,3.626983676966945,1.8552707464765987,-0.8136884090604357,2.8178122201329643,2.6119299003778274]

        resp = service(req.model_name,req.urdf_param_name,req.joint_names,req.joint_positions)

    except rospy.ServiceException, e:
        print("Service call failed: %s" %e)
