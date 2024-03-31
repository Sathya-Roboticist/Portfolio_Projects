#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
import time

current_pitch = 0.0
# pid_effort = Float64
class PIDController:
    def __init__(self, Kp, Ki, Kd, min_output, max_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_output = min_output
        self.max_output = max_output
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, current_value):
        error = setpoint - current_value
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = min(max(output, self.min_output), self.max_output)
        return output

class DepthController:
    global pid_effort
    def __init__(self):
        rospy.init_node('depth_pid_controller')

        self.controller = PIDController(Kp=13, Ki=2.0, Kd=0.6870611608028412, min_output=0.7, max_output=1.2)
        self.depth_setpoint_sub = rospy.Subscriber('/auv3/setpoint_depth1', Float64, self.depth_setpoint_callback)
        self.depth_sub = rospy.Subscriber('/neerakshi/dvl_link_altitude_modified', Float64, self.depth_callback)
        self.pitch_sub = rospy.Subscriber('/auv3/orientation', Vector3, self.pitch_callback)
        self.thruster_pub_f = rospy.Publisher('/neerakshi/control/thruster/forward_up', Float64, queue_size=10)
        self.thruster_pub_r = rospy.Publisher('/neerakshi/control/thruster/rear_up', Float64, queue_size=10)
        self.depth_setpoint = 0.0
        self.autotune_stage = 0
        self.rear_thrust = 0.0  # Initial thrust for rear thruster
        self.front_thrust = 0.0  # Initial thrust for front thruster


    def pitch_callback(self, data):
        global current_pitch
        current_pitch = data.y

    def depth_setpoint_callback(self, data):
        self.depth_setpoint = data.data

    def depth_callback(self, data):
        current_depth = data.data
        
        if self.autotune_stage == 0:
            # Start autotuning process by oscillating the system
            self.autotune_stage = 1
            self.autotune_start_time = time.time()
            self.controller.Kp = 1.0  # Start with a low gain
            self.initial_depth = current_depth
            self.prev_depth = current_depth

        elif self.autotune_stage == 1:
            # Measure oscillation period and amplitude
            if (time.time() - self.autotune_start_time) > 5:  # Change this duration as needed
                oscillation_period = time.time() - self.autotune_start_time
                oscillation_amplitude = abs(self.initial_depth - self.prev_depth) / 2.0
                self.controller.Kp = 13  # Reset Kp to stop oscillation
                self.controller.Ki = 0.6 / oscillation_period  # Calculate Ki
                self.controller.Kd = 0.125 * oscillation_period  # Calculate Kd
                print(f"Kp: {self.controller.Kp}, Ki: {self.controller.Ki}, Kd: {self.controller.Kd}")
                self.autotune_stage = 2  # Autotuning complete

        elif self.autotune_stage == 2:
            # Run PID with tuned parameters
            pid_effort = self.controller.compute(self.depth_setpoint, current_depth)
            
            if current_pitch > 0:
                rear_thrust = 1.2 # Maximum thrust
                front_thrust = 1.2  # Minimum thrust
                print("i am here")
            elif current_pitch < 0:
                # Adjust thrusters if AUV is tilting backward
                rear_thrust = 0.7
                front_thrust = 0.7
            else:
                # No tilt, balanced pitch
                rear_thrust = 0.0
                front_thrust = 0.0
            
            self.thruster_pub_f.publish(front_thrust)
            self.thruster_pub_r.publish(rear_thrust)
            self.thruster_pub_f.publish(pid_effort)
            self.thruster_pub_r.publish(pid_effort)

        self.prev_depth = current_depth


def main():
    controller = DepthController()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
