#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState

def main():
    rospy.init_node("set_triton_pose", anonymous=True)

    model_name = rospy.get_param("~model_name", "triton")

    # target pose from get_model_state output
    x = rospy.get_param("~x", -3.4911271113192655)
    y = rospy.get_param("~y", -3.5158118437849675)
    z = rospy.get_param("~z", 0.0)
    qx = rospy.get_param("~qx", -1.4769940887105835e-06)
    qy = rospy.get_param("~qy",  1.048504801470323e-05)
    qz = rospy.get_param("~qz", -0.00023837490883773285)
    qw = rospy.get_param("~qw",  0.9999999715326422)

    rospy.loginfo("Waiting for /gazebo/get_model_state and /gazebo/set_model_state...")
    rospy.wait_for_service("/gazebo/get_model_state")
    rospy.wait_for_service("/gazebo/set_model_state")
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    # Wait until the model actually exists
    rospy.loginfo(f"Waiting for model '{model_name}' to exist in Gazebo...")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        resp = get_state(model_name, "world")
        if resp.success:
            break
        rate.sleep()

    st = ModelState()
    st.model_name = model_name
    st.pose.position.x = x
    st.pose.position.y = y
    st.pose.position.z = z
    st.pose.orientation.x = qx
    st.pose.orientation.y = qy
    st.pose.orientation.z = qz
    st.pose.orientation.w = qw
    st.twist.linear.x = st.twist.linear.y = st.twist.linear.z = 0.0
    st.twist.angular.x = st.twist.angular.y = st.twist.angular.z = 0.0
    st.reference_frame = "world"

    ok = set_state(st).success
    rospy.loginfo(f"SetModelState sent. success={ok}")

    rospy.sleep(0.2)

if __name__ == "__main__":
    main()