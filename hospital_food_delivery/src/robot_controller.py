#!/usr/bin/env python3
# license removed for brevity

import rospy
import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import time
import argparse
import json
import numpy as np
import math
import sys


# as opposed to from an external json
class InMemoryMissionEnvironment():
    def __init__(self):
        
        self.places = {
            "H": [-.2,-1.5],
            "A": [4,-1],
            "B": [4,-7],
            "kitchen": [0.5,-7],
            "C": [4,3],
            "table": [2,6],
        }
    
    def get_variable(self, v):
        with open('/root/catkin_ws/src/hospital_food_delivery/worlds/status.json', 'r') as file:
            data = json.load(file)
            return data[v]
    
    def get_local_mission(self, ns):
        # Implement function to return the local mission plans
        # and return the dict of the mission
        local_missions = {
            "turtlebot1" : {
                "main": ["nav_A","nav_B","nav_kitchen","wait_5","nav_B","nav_C","adapt|deliver"],
                # "main": ["nav_A","nav_C","adapt|deliver"],
                "deliver": {
                    "selectors": ["self_care_ability","alone"],
                    "plans": [
                        {"name": "awake", 
                         "sequence": ["nav_table","wait_5","nav_C","nav_A","nav_H"], 
                         "trigger": lambda args : args[0] == 1 or args[1] == 0
                         },
                        {"name": "asleep", 
                         "sequence": ["call_turtlebot2","listen","nav_C","nav_A","nav_H"], 
                         "trigger": lambda args : args[0] == 0 and args[1] == 1
                        }
                    ],
                }
            },
            "turtlebot2" : {
                "main": ["listen","call_turtlebot1", "nav_table"]
            }
            
        }
        return local_missions[ns]

    
    def get_place_location(self,name):
        if name in self.places:
            return self.places[name]
        rospy.loginfo(f"FATAL: Place {name} not found")
        return None


class RobotController():
    def __init__(self, prefix):
        self.ns = prefix
        rospy.loginfo(self.ns+'/cmd_vel')
        self.vel_publisher = rospy.Publisher(self.ns+'/cmd_vel',Twist,queue_size=10)
        rospy.init_node(f'robot_controller_{self.ns[-1]}')
        self.odom = None 
        self.setpoint = Twist()
        self.odom_subscriber = rospy.Subscriber(self.ns+'/odom', Odometry, self.update_odom)
        self.comm_subscriber = rospy.Subscriber('bot_comms', String, self.receive_comms_msg)
        self.comm_publisher = rospy.Publisher('bot_comms', String, queue_size=10)
        self.mission_env = InMemoryMissionEnvironment()
        self.init_skills()
        self.control_params = {
            "k_linear": 0.12,
            "k_angular": 0.34,
            "max_linear": 0.7,
            "dist_threshold": 0.8,
            "sample_period": 0.35
        }
        self.message_received = False
        # Set velocity timer
        vel_timer = rospy.Timer(rospy.Duration(self.control_params["sample_period"]), self.set_velocity)

    def receive_comms_msg(self,msg):
        rospy.loginfo(f"Message read in the robot comms: {msg.data}")
        self.message_received = msg.data.find(self.ns) >= 0


    # helper functions
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    

    
    def update_odom(self, msg):
        self.odom = msg.pose
        quat = (
                self.odom.pose.orientation.x,
                self.odom.pose.orientation.y,
                self.odom.pose.orientation.z,
                self.odom.pose.orientation.w,
        )
        self.angs = self.euler_from_quaternion(quat)
    
    def stop_cmd(self):
        self.setpoint = Twist()

    def set_velocity(self, c):
        self.vel_publisher.publish(self.setpoint)

    def get_current_state(self):
        # return current state (x,y,theta)
        if self.odom is not None:
            return float(self.odom.pose.position.x), float(self.odom.pose.position.y), self.angs[2]  
        return 0.0,0.0,0.0

    def wrap_angle(self, previous_angle, new_angle):
        """
        Adjust the new_angle based on the previous_angle to maintain continuity.
        Assumes angles are in radians.
        """
        angle_difference = new_angle - previous_angle
        if angle_difference > math.pi:
            new_angle -= 2 * math.pi
        elif angle_difference < -math.pi:
            new_angle += 2 * math.pi
        return new_angle

    def go_to_goal(self, x_goal, y_goal):
        rospy.loginfo(f"Starting the go to goal to {x_goal}|{y_goal}...")
        x,y,yaw = self.get_current_state()
        x_goal = float(x_goal)
        y_goal = float(y_goal)
        # rospy.loginfo("\n\n\n",type(x),type(y))
        vel_msg = Twist()
        p_ang = 0.0
        p_yaw = 0.0
        while True:
            x,y,yaw = self.get_current_state()
            yaw = self.wrap_angle(p_yaw, yaw)
            p_yaw = yaw
            k_linear = self.control_params['k_linear']
            distance = abs(math.sqrt((x_goal-x)**2 + (y_goal-y)**2))
            linear_speed = min(self.control_params['max_linear'],distance * k_linear)
            # pos_msg = f"The current pos: {x},{y}  | Desired pos: {x_goal},{y_goal} | D: {distance}"
            # rospy.loginfo(pos_msg)
            k_angular = self.control_params['k_angular']
            desired_angle_goal = self.wrap_angle(p_ang, math.atan2(y_goal-y,x_goal-x))
            p_ang = desired_angle_goal
            # pos_msg = f"The current angle: {math.degrees(yaw)}  | Desired: {math.degrees(desired_angle_goal)} | Da: {math.degrees(desired_angle_goal-yaw)}"
            # rospy.loginfo(pos_msg)
            angular_speed = (desired_angle_goal-yaw)*k_angular
            vel_msg.linear.x = linear_speed
            vel_msg.angular.z = angular_speed
            self.setpoint = vel_msg
            # rospy.loginfo(f"The distance to the goal is {distance}")
            if distance < self.control_params['dist_threshold']:
                self.setpoint = Twist()
                return
    
    def turn_motion(self, angle, duration):
        #Implement the turn for some seconds in between some angles
        v = Twist()
        v.angular.z = angle
        self.setpoint = v
        rospy.loginfo(f"Setting spinning motion for {duration} secs.")
        rospy.sleep(duration)
        self.stop_cmd()
        

    def init_skills(self):
        self.skill_exec = {
            "nav" : self.navigate,
            "collect": self.collect,
            "deliver": self.deliver,
            "wait": self.wait,
            "call": self.call,
            "listen": self.listen_for_msg
        }

    # skill handlers
    def wait(self, kw):
        n = int(kw.split("_")[1])
        rospy.sleep(n)

    def listen_for_msg(self,kw):
        rospy.loginfo(f"Waiting for a message for {self.ns}")
        while not self.message_received:
            rospy.sleep(1)
    
    def call(self, kw):
        s = String(data=f"calling : {kw}")
        self.comm_publisher.publish(s)

    def navigate(self, kw):
        # decipher keyword
        place = kw.split("_")[1]
        self.go_to_goal(self.mission_env.places[place][0],self.mission_env.places[place][1])
    
    def collect(self, kw):
        self.turn_motion(0.1,3)

    
    def deliver(self, kw):
        self.turn_motion(0.1,3)

    
    def execute_local_plan(self):
        rospy.loginfo(f"Starting the Local Mission Plan for {self.ns}")
        plan = self.mission_env.get_local_mission(self.ns)
        for activity in plan['main']:
            act_data = activity.split('|')
            act = act_data[0]
            rospy.loginfo(f"Doing activity: {act}")
            if act != 'adapt':
                sk = act.split('_')[0]
                self.skill_exec[sk](act)
            
            else: # Adapting 
                aplan = plan[act_data[1]]
                # var_value = self.mission_env.get_variable(aplan['selector'])
                active_plan = None
                selector_values = [self.mission_env.get_variable(v) for v in  aplan['selectors']]

                for p in aplan['plans']:
                    # if p['limits'][0] <= var_value <= p['limits'][1]:
                        # active_plan = p
                        # break
                    r = p['trigger'](selector_values)
                    rospy.loginfo(f"The trigger function for {p['name']} returned {r}") 
                    rospy.loginfo(f"The selector values are: {selector_values}")
                    if p['trigger'](selector_values):
                        active_plan = p
                        break
                rospy.loginfo(f"Adaptive plan selected: {active_plan}")
                if active_plan is not None:
                    for aact in active_plan["sequence"]:
                        rospy.loginfo(f"Doing activity: {aact}")
                        sk = aact.split('_')[0]
                        self.skill_exec[sk](aact)
        
        rospy.loginfo("Local Plan executed successfully!!")
            


def main(args):
    rospy.loginfo(f"The args received are :{args}")
    # ns = args[2]
    if len(args) >= 3:
        ns = "turtlebot" + args[2]
    else:
        ns = "turtlebot1"
    r = RobotController(ns)
    rospy.loginfo(f"The args received are :{args}")
    r.execute_local_plan()    
    rospy.spin()

if __name__ == "__main__":
    main(sys.argv)