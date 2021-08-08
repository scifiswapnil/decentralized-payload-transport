#!/usr/bin/env python3
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

rospy.init_node('manipulator')
attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
attach_srv.wait_for_service()
detach_srv.wait_for_service()

req = AttachRequest()
req.model_name_1 = "jackal0"
req.link_name_1 = "jackal0/base_link"
req.model_name_2 = "pallet"
req.link_name_2 = "connector_bar_one"
attach_srv.call(req)

req = AttachRequest()
req.model_name_1 = "jackal2"
req.link_name_1 = "jackal2/base_link"
req.model_name_2 = "pallet"
req.link_name_2 = "connector_bar_two"
attach_srv.call(req)