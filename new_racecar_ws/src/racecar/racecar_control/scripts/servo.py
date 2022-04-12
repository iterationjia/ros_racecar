#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0

def set_throttle_steer(data):

    global flag_move
    pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    # Velocity is in terms of radians per second.
    # Want to go 1 m/s with a wheel of radius 0.05m. This translates to 19.97 radians per second, roughly 20.
    # However, at a multiplication factor of 20 speed is half of what it should be, so doubled to 40.
    throttle = data.drive.speed * 40.0
    steer = data.drive.steering_angle

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)
    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)

def limsteer(data,maxdata):
    if data>0 and data > maxdata:
        data = maxdata
    elif data<0 and math.fabs(data) > maxdata:
        data = maxdata
    return data


def set_speed(data):
    global flag_move
    
    pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)
    
    # Velocity is in terms of radians per second.
    # Want to go 1 m/s with a wheel of radius 0.05m. This translates to 19.97 radians per second, roughly 20.
    # However, at a multiplication factor of 20 speed is half of what it should be, so doubled to 40.
    x = data.linear.x  
    z = data.angular.z
    L = 0.335   
    T = 0.305   
    if z!=0 and x!=0:
        r=math.fabs(x/z)

        rL_rear = r-(math.copysign(1,z)*(T/2.0))   
        rR_rear = r+(math.copysign(1,z)*(T/2.0))
        rL_front = r-(math.copysign(1,z)*(T/2.0))
        rR_front = r+(math.copysign(1,z)*(T/2.0))
        vL_rear = x*rL_rear/r
        vR_rear = x*rR_rear/r
        vL_front = x*rL_front/r
        vR_front = x*rR_front/r
        anL_front = math.atan2(L,rL_front)*math.copysign(1,z)
        anR_front = math.atan2(L,rR_front)*math.copysign(1,z)
        
    else:
        vL_rear = x
        vR_rear = x
        vL_front =x
        vR_front =x
        anL_front = z
        anR_front = z
    anL_front = limsteer(anL_front,3.0) 
    anR_front = limsteer(anR_front,3.0)

    pub_vel_left_rear_wheel.publish(vL_rear*45)
    pub_vel_right_rear_wheel.publish(vR_rear*45)
    pub_vel_left_front_wheel.publish(vL_front*45)
    pub_vel_right_front_wheel.publish(vR_front*45)
    pub_pos_left_steering_hinge.publish(anL_front)
    pub_pos_right_steering_hinge.publish(anR_front)

def servo_commands():
    rospy.init_node('servo_commands', anonymous=True)
    #rospy.Subscriber("/racecar/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)     
    rospy.Subscriber("/motor_output", Twist, set_speed)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
