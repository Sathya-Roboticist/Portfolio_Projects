#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32, String, Float64
from geometry_msgs.msg import Vector3
from math import copysign
import time
import math

# publishers

pub_fwd_s = rospy.Publisher('/neerakshi/control/thruster/forward_side', Float64, queue_size=10)
pub_rear_s = rospy.Publisher('/neerakshi/control/thruster/rear_side', Float64, queue_size=10)
pub_surge = rospy.Publisher('/neerakshi/control/thruster/surge', Float64, queue_size=10)

yaw_thrust_pub = rospy.Publisher('auv3/yaw_msg', String, queue_size=10)
yaw_plant_pub = rospy.Publisher("auv3/plant/yaw", Float64, queue_size=10)
vel_plant_pub = rospy.Publisher("auv3/plant/velocity", Float64, queue_size=10)

pi = math.pi


# plants
yaw_plant_msg = 0.0
velocity_plant_msg = 0.0

# setpoints
yaw_setpoint_msg = 0.0
vel_setpoint_msg = 0.0
velocity_setpoint = 0.0


def yaw_setpoint_callback(data):
    global yaw_setpoint_msg
    yaw_setpoint_msg = data.data
    print("Yaw_setpoint = " + str(yaw_setpoint_msg))

def vel_setpoint_callback(data):
    global vel_setpoint_msg
    vel_setpoint_msg = data.data
    print("Vel_setpoint = " + str(vel_setpoint_msg))

def velocity_callback(data):
    velocity_plant_msg = data.data
    vel_plant_pub.publish(velocity_plant_msg)
    
    
def compare_angles(x_plant, x_setpoint):  # 0 -0.05
    d = abs(x_setpoint-x_plant)
    if(d > pi):
        d = -1
    else:
        d = 1
    return d


def orientation_callback(data):
    global pitch_plant_msg
    global yaw_plant_msg
    pitch_plant_msg = data.y
    yaw_plant_msg = data.z
    yaw_plant_pub.publish(yaw_plant_msg)


'''Main Control logics are here'''

# Yaw Control

def yaw_controller(data):
    global yaw_plant_msg, yaw_setpoint_msg, prev_yaw_msg, yaw_msg, yaw_effort_msg
    yaw_msg = 0.0
    diff = compare_angles(yaw_plant_msg, yaw_setpoint_msg)

    if (yaw_setpoint_msg - yaw_plant_msg >= 0 ):
        yaw_effort_msg = 0.4
    else:
        yaw_effort_msg = -0.4
    
    if(diff < 0.0):
        yaw_msg += yaw_effort_msg
    if(diff > 0.0):
        yaw_msg -= yaw_effort_msg

    yaw_thrust_pub.publish(str(yaw_msg))
    pub_fwd_s.publish(yaw_msg)
    time.sleep(0.1)

# Velocity Control
def velocity_controller(data):
    surge_msg = 0.0
    velocity_effort_msg = data.data
    surge_msg+=velocity_effort_msg
    pub_surge.publish(round(surge_msg))
    pub_surge.publish(surge_msg)

if __name__ == "__main__":
    rospy.init_node('depth_pid_controller', anonymous=True)
    r = rospy.Rate(1)
    # Subscribers

    # Subscribers for data acquisition from sensors
    oreintation_sub = rospy.Subscriber('/auv3/orientation', Vector3, orientation_callback)
    velocity_sub = rospy.Subscriber("/neerakshi/velocity",Float64, velocity_callback)
    
    # Subscribers for controller setpoint
    yaw_setpoint_sub = rospy.Subscriber("auv3/yaw_setpoint", Float64, yaw_setpoint_callback)
    vel_setpoint_sub = rospy.Subscriber("auv3/controller/velocity_setpoint", Float64, vel_setpoint_callback)

    # Subscribers for efforts from PID Controller
    yaw_effort_sub = rospy.Subscriber("auv3/controller/yaw_effort", Float64, yaw_controller)
    velocity_effort_sub = rospy.Subscriber('auv3/velocity_effort',Float64,velocity_controller)
  
    print("Controller Script Initialized!")
    rospy.spin()