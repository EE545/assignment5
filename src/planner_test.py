#!/usr/bin/env python

import rospy
import numpy as np
from assignment5.srv import *
import Utils
from nav_msgs.srv import GetMap

PLANNER_SERVICE_TOPIC = '/planner_node/get_plan'

# Testing pose sets:
SOURCE1 = [156, 1080, 0.0]
TARGET1 = [519, 828, 0.0]

SOURCE2 = [765, 504, 0.0]
TARGET2 = [1608, 729, 0.0]

SOURCE3 = [2328, 462, 0.0]
TARGET3 = [456, 732, 0.0]

if __name__ == '__main__':

    rospy.init_node('planner_test', anonymous=True)

    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    map_info = rospy.ServiceProxy(map_service_name, GetMap)().map.info

    rospy.wait_for_service(PLANNER_SERVICE_TOPIC)
    get_plan = rospy.ServiceProxy(PLANNER_SERVICE_TOPIC, GetPlan)

    try:
        resp = get_plan(Utils.map_to_world(SOURCE1, map_info), Utils.map_to_world(TARGET1, map_info))
        print(np.array(resp.plan).reshape(-1, 3))
        print(resp.success)
    except rospy.ServiceException as e:
        print('Service call failed: %s' % e)

