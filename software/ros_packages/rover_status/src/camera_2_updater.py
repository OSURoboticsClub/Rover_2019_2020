#!/usr/bin/env python

# import rospy
# from system_statuses.msg import Camera2Changer


# def camera_2_changer():
#    pub = rospy.Publisher('camera_2_changer_chatter', Camera2Changer, queue_size=10)
#    rospy.init_node('camera_2_changer_talker', anonymous=True)
#    # r = rospy.sleep(10)  # 10hz
#    msg = Camera2Changer()
#    msg.camera_2_value = 0

#    while not rospy.is_shutdown():
#        msg.camera_2_value = (msg.camera_2_value + 1) % 2
#        rospy.loginfo(msg)
#        pub.publish(msg)
        # r.sleep()
##         rospy.sleep(2.)


#if __name__ == '__main__':
#    try:
#        camera_2_changer()
#    except rospy.ROSInterruptException:
#        pass
