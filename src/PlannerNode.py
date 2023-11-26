#!/usr/bin/env python

import rospy 
import numpy as np
from threading import Lock
import sys 

from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, PoseArray
from assignment5.srv import *

from HaltonPlanner import HaltonPlanner
from HaltonEnvironment import HaltonEnvironment
import GraphGenerator
import Utils

try:
    xrange
except NameError:
    xrange = range


class PlannerNode(object):

  def __init__(self, map_service_name, 
                     halton_points, 
                     disc_radius,
                     collision_delta,                      
                     source_topic,
                     target_topic,
                     pub_topic,
                     service_topic,
                     car_width,
                     car_length):
    
    print("[Planner Node] Getting map from service...")
    rospy.wait_for_service(map_service_name)
    self.map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
    print("[Planner Node] ...got map")
    
    print("[Planner Node] Generating graph file...")
    graph_file = GraphGenerator.generate_graph_file(self.map_msg, halton_points, disc_radius, car_width, car_length, collision_delta)
    print("[Planner Node] ..graph generated")
    
    self.environment = HaltonEnvironment(self.map_msg, graph_file, None, None, car_width, car_length, disc_radius, collision_delta)
    self.planner = HaltonPlanner(self.environment)
    
    self.source_pose = None
    self.source_updated = False
    self.source_yaw = None
    self.source_lock = Lock()
    self.target_pose = None
    self.target_updated = False
    self.target_yaw = None
    self.target_lock = Lock()
    
    self.cur_plan = None
    self.plan_lock = Lock()
    
    self.orientation_window_size = 21
    
    if pub_topic is not None:
      self.plan_pub = rospy.Publisher(pub_topic, PoseArray, queue_size=1)  
      self.source_sub = rospy.Subscriber(source_topic, 
                                         PoseWithCovarianceStamped, 
                                         self.source_cb,
                                         queue_size=1)
      self.target_sub = rospy.Subscriber(target_topic, 
                                         PoseStamped, 
                                         self.target_cb,
                                         queue_size=1)          
    else:
      self.plan_pub = None
                                         
    if service_topic is not None:
      self.plan_service = rospy.Service(service_topic, GetPlan, self.get_plan_cb)
    else:
      self.plan_service = None
    
    
    print('[Planner Node] Ready to plan')
    
  def get_plan_cb(self, req):
    self.source_lock.acquire()
    self.source_pose = [req.source.pose.position.x,
                        req.source.pose.position.y]
    self.source_yaw = Utils.quaternion_to_angle(req.source.pose.orientation)
    self.source_updated = True
    self.source_lock.release()    
    
    self.target_lock.acquire()
    self.target_pose = [req.target.pose.position.x,
                        req.target.pose.position.y]
    self.target_yaw = Utils.quaternion_to_angle(req.target.pose.orientation)
    self.target_updated = True
    self.target_lock.release()  
    
    self.plan_lock.acquire()
    self.update_plan()
    
    if self.cur_plan is not None:
      gpr = GetPlanResponse()
      gpr.plan.header.frame_id = '/map'
      gpr.plan.header.stamp = rospy.Time.now()
      for i in xrange(len(self.cur_plan)):
        pose = Pose()
        pose.position.x = self.cur_plan[i][0]
        pose.position.y = self.cur_plan[i][1]
        pose.position.z = 0.0
        pose.orientation = Utils.angle_to_quaternion(self.cur_plan[i][2])
        gpr.plan.poses.append(pose)    
      self.plan_lock.release()
      return gpr
    else:          
      self.plan_lock.release() 
      return 
    
  def source_cb(self, msg):
    self.source_lock.acquire()
    
    print('[Planner Node] Got new source')
    self.source_pose = [msg.pose.pose.position.x,
                        msg.pose.pose.position.y]
    self.source_yaw = Utils.quaternion_to_angle(msg.pose.pose.orientation)
    self.source_updated = True
    
    self.source_lock.release()
    
  def target_cb(self, msg):  
    self.target_lock.acquire()
    
    print('[Planner Node] Got new target')
    self.target_pose = [msg.pose.position.x,
                        msg.pose.position.y]
    self.target_yaw = Utils.quaternion_to_angle(msg.pose.orientation)
    self.target_updated = True
    
    self.target_lock.release()    
    
  def publish_plan(self, plan):
    pa = PoseArray()
    pa.header.frame_id = "/map"
    for i in xrange(len(plan)):
      config = plan[i]
      pose = Pose()
      pose.position.x = config[0]
      pose.position.y = config[1]
      pose.position.z = 0.0
      pose.orientation = Utils.angle_to_quaternion(config[2])
      pa.poses.append(pose)
    self.plan_pub.publish(pa) 

      
  def add_orientation(self, plan):
    plan = np.array(plan)

    oriented_plan = np.zeros((plan.shape[0],3),dtype=np.float)
    oriented_plan[:,0:2] = plan[:,:]
 
    if plan.shape[0] >= 2:  
      oriented_plan[0,2] = self.source_yaw
      oriented_plan[oriented_plan.shape[0]-1, 2] = self.target_yaw   
          
      plan_diffs = np.zeros(plan.shape, np.float)
      plan_diffs[0:plan_diffs.shape[0]-1] = plan[1:plan.shape[0]]-plan[0:plan.shape[0]-1]
      plan_diffs[plan_diffs.shape[0]-1] = np.array([np.cos(self.target_yaw), np.sin(self.target_yaw)], dtype=np.float) 
    
      avg_diffs = np.empty(plan_diffs.shape, dtype=np.float)
      for i in xrange(plan_diffs.shape[0]):
        avg_diffs[i] = np.mean(plan_diffs[np.max((0,i-self.orientation_window_size/2)):
                                          np.min((plan_diffs.shape[0]-1, i+self.orientation_window_size/2+1))],
                               axis=0)

      oriented_plan[1:oriented_plan.shape[0]-1,2] = np.arctan2(avg_diffs[1:oriented_plan.shape[0]-1,1],
                                                               avg_diffs[1:oriented_plan.shape[0]-1,0])
   
    elif plan.shape[0] == 2:
      oriented_plan[:,2] = np.arctan2(plan[1,1]-plan[0,1],plan[1,0]-plan[0,0])

    return oriented_plan
      
  def update_plan(self):
    self.source_lock.acquire()
    self.target_lock.acquire()
    if self.source_pose is not None:
      source_pose = np.array(self.source_pose).reshape(2)
    if self.target_pose is not None:
      target_pose = np.array(self.target_pose).reshape(2)
    replan = ((self.source_updated or self.target_updated) and
              (self.source_pose is not None and self.target_pose is not None))
    self.source_updated = False
    self.target_updated = False
    self.source_lock.release()
    self.target_lock.release()
    
    if replan:
      if(np.abs(source_pose-target_pose).sum() < sys.float_info.epsilon):
        print('[Planner Node] Source and target are the same, will not plan')
        return

      if not self.environment.manager.get_state_validity(source_pose):
        print('[Planner Node] Source in collision, will not plan')
        return

      if not self.environment.manager.get_state_validity(target_pose):
        print('[Planner Node] Target in collision, will not plan')
        return

      print('[Planner Node] Inserting source and target')
      self.environment.set_source_and_target(source_pose, target_pose)

      print('[Planner Node] Computing plan...')
      self.cur_plan = self.planner.plan()    
      
      if self.cur_plan is not None:
        self.cur_plan = self.add_orientation(self.cur_plan)
        #self.cur_plan = self.planner.post_process(self.cur_plan, 5)
        print('[Planner Node] ...plan complete')
      else:
        print('[Planner Node] ...could not compute a plan')
      

    if (self.cur_plan is not None) and (self.plan_pub is not None):
      self.publish_plan(self.cur_plan)  
    
if __name__ == '__main__':
  rospy.init_node('planner_node', anonymous=True)
  
  map_service_name = rospy.get_param("~static_map", "static_map")
  halton_points = rospy.get_param("~halton_points", 500)
  disc_radius = rospy.get_param("~disc_radius", 3)
  collision_delta = rospy.get_param("~collision_delta", 0.05)  
  source_topic = rospy.get_param("~source_topic" , "/initialpose")
  target_topic = rospy.get_param("~target_topic", "/move_base_simple/goal")
  pub_topic = rospy.get_param("~pub_topic", None)
  service_topic = rospy.get_param("~service_topic", None)
  car_width = rospy.get_param("/car_kinematics/car_width", 0.33)
  car_length = rospy.get_param("/car_kinematics/car_length", 0.33)

  
  pn = PlannerNode(map_service_name, 
                   halton_points, 
                   disc_radius,
                   collision_delta,                    
                   source_topic,
                   target_topic,
                   pub_topic,
                   service_topic,
                   car_width,
                   car_length)
                   
  while not rospy.is_shutdown():
    if pub_topic is not None:
      pn.plan_lock.acquire()
      pn.update_plan()
      pn.plan_lock.release()
    rospy.sleep(1.0) 

